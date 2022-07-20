// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#include <asp/Core/InterestPointMatching.h>
#include <vw/Math/GaussianClustering.h>
#include <vw/Math/RANSAC.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Stereo/StereoModel.h>

using namespace vw;

namespace asp {

int g_ip_num_errors = 0;
Mutex g_ip_mutex;
  
// Class EpipolarLinePointMatcher
  
EpipolarLinePointMatcher::EpipolarLinePointMatcher(bool   single_threaded_camera,
                                                   double uniqueness_threshold,
                                                   double epipolar_threshold,
                                                   vw::cartography::Datum const& datum):
  m_single_threaded_camera(single_threaded_camera), m_uniqueness_threshold(uniqueness_threshold),
  m_epipolar_threshold(epipolar_threshold), m_datum(datum) {
  // Detect some problems before they result in strange math errors
  if (epipolar_threshold < 1)
    vw_throw( ArgumentErr() << "EpipolarLinePointMatcher: epipolar threshold is < 1.\n" );
  if (uniqueness_threshold < 0.1)
    vw_throw( ArgumentErr() << "EpipolarLinePointMatcher: uniqueness threshold is < 0.1.\n" );
  if (uniqueness_threshold > 0.99)
    vw_throw( ArgumentErr() << "EpipolarLinePointMatcher: uniqueness threshold is > 0.99.\n" );
}
  
Vector3 EpipolarLinePointMatcher::epipolar_line(Vector2 const& feature,
                                                cartography::Datum const& datum,
                                                camera::CameraModel* cam_ip,
                                                camera::CameraModel* cam_obj,
                                                bool & success) {
  success = true;

  // Watch out for errors thrown when projecting into the camera
  try{

    // Intersect the interest point pixel with the datum
    Vector3 p0 = cartography::datum_intersection( datum, cam_ip, feature );

    if (p0 == Vector3()){ // No intersection
      success = false;
      return Vector3();
    }

    Vector3 p1  = p0 + 10*cam_ip->pixel_to_vector( feature ); // Extend the point below the datum
    Vector2 ep0 = cam_obj->point_to_pixel( p0 ); // Project the intersection and extension into the other camera
    Vector2 ep1 = cam_obj->point_to_pixel( p1 );
    Matrix<double> matrix( 2, 3 );
    select_col( matrix, 2 ) = Vector2(1,1);
    matrix(0,0) = ep0.x();
    matrix(0,1) = ep0.y();
    matrix(1,0) = ep1.x();
    matrix(1,1) = ep1.y();

    if (matrix != matrix){ // Got back NaN values. Can't proceed.
      success = false;
      return Vector3();
    }

    // If the input matrix is bad this can result in some weird errors!
    Matrix<double> nsp = nullspace( matrix );
    if (nsp.cols() <= 0 || nsp.rows() <= 0){ // Failed to find the nullspace
      success = false;
      return Vector3();
    }

    return select_col(nsp,0);

  } catch (std::exception const& e) {
    // Turn this off, it can be verbose
    //Mutex::Lock lock( g_ip_mutex );
    //g_ip_num_errors++;
    //if (g_ip_num_errors < 100) {
    //  vw_out(ErrorMessage) << e.what() << std::endl;
    //}else if (g_ip_num_errors == 100) {
    //  vw_out() << "Will print no more error messages about failing to find epipolar line.\n";
    //}
  }

  success = false;
  return Vector3();
}

// TODO(oalexan1): Use std::abs() instead of fabs(). May change the result.
double EpipolarLinePointMatcher::distance_point_line(Vector3 const& line,
                                                     Vector2 const& point) {
  return fabs(line.x() * point.x() + line.y() * point.y() + line.z() ) /
    norm_2(subvector(line, 0, 2));
}

// Local class definition -----
class EpipolarLineMatchTask : public Task, private boost::noncopyable {
  typedef ip::InterestPointList::const_iterator IPListIter;
  bool                            m_single_threaded_camera;
  bool                            m_use_uchar_tree;
  math::FLANNTree<float        >& m_tree_float;
  math::FLANNTree<unsigned char>& m_tree_uchar;
  IPListIter                      m_start, m_end;
  ip::InterestPointList const&    m_ip_other;
  camera::CameraModel            *m_cam1, *m_cam2;
  EpipolarLinePointMatcher const& m_matcher;
  Mutex&                          m_camera_mutex;
  std::vector<size_t>::iterator   m_output;
public:
  EpipolarLineMatchTask( bool single_threaded_camera,
                         bool use_uchar_tree,
                         math::FLANNTree<float        >& tree_float,
                         math::FLANNTree<unsigned char>& tree_uchar,
                         ip::InterestPointList::const_iterator start,
                         ip::InterestPointList::const_iterator end,
                         ip::InterestPointList const& ip2,
                         camera::CameraModel* cam1,
                         camera::CameraModel* cam2,
                         EpipolarLinePointMatcher const& matcher,
                         Mutex& camera_mutex,
                         std::vector<size_t>::iterator output ) :
    m_single_threaded_camera(single_threaded_camera),
    m_use_uchar_tree(use_uchar_tree), m_tree_float(tree_float), m_tree_uchar(tree_uchar),
    m_start(start), m_end(end), m_ip_other(ip2),
    m_cam1(cam1), m_cam2(cam2),
    m_matcher( matcher ), m_camera_mutex(camera_mutex), m_output(output) {}

  void operator()() {

    const size_t NUM_MATCHES_TO_FIND = 10;
    Vector<int   > indices  (NUM_MATCHES_TO_FIND);
    Vector<double> distances(NUM_MATCHES_TO_FIND);

    for ( IPListIter ip = m_start; ip != m_end; ip++ ) {
      Vector2 ip_org_coord = Vector2( ip->x, ip->y );
      Vector3 line_eq;

      // Find the equation that describes the epipolar line
      bool found_epipolar = false;
      if (m_single_threaded_camera){
        // ISIS camera is single-threaded
        Mutex::Lock lock( m_camera_mutex );
        line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, m_cam1, m_cam2, found_epipolar);
      }else{
        line_eq = m_matcher.epipolar_line( ip_org_coord, m_matcher.m_datum, m_cam1, m_cam2, found_epipolar);
      }

      if (!found_epipolar) {
        *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
        continue; // Skip to the next IP
      }

      // Use FLANN tree to find the N nearest neighbors according to the IP region descriptor?
      std::vector<std::pair<float,int> > kept_indices;
      kept_indices.reserve(NUM_MATCHES_TO_FIND);

      // Call the correct FLANN tree for the matching type
      size_t num_matches_valid = 0;
      if (m_use_uchar_tree) {
        vw::Vector<unsigned char> uchar_descriptor(ip->descriptor.size());
        for (size_t i=0; i<ip->descriptor.size(); ++i)
          uchar_descriptor[i] = static_cast<unsigned char>(ip->descriptor[i]);
        num_matches_valid = m_tree_uchar.knn_search( uchar_descriptor, indices, distances, NUM_MATCHES_TO_FIND );
      } else {
        num_matches_valid = m_tree_float.knn_search( ip->descriptor, indices, distances, NUM_MATCHES_TO_FIND );
      }

      if (num_matches_valid < 1) {
        *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
        continue; // Skip to the next IP
      }

      //vw_out() << "For descriptor: " << ip->descriptor << std::endl;
      //vw_out() << num_matches_valid << " Best match distances: " << distances << std::endl;
      //vw_out() << "Indices: " << indices << std::endl;

      // Loop through the N "nearest" points and keep only the ones within
      //   m_matcher.m_epipolar_threshold pixel distance from the epipolar line
      const double EPIPOLAR_BAND_EXPANSION = 200;
      double small_epipolar_threshold = m_matcher.m_epipolar_threshold;
      double large_epipolar_threshold = small_epipolar_threshold + EPIPOLAR_BAND_EXPANSION;
      for ( size_t i = 0; i < num_matches_valid; i++ ) {
        IPListIter ip2_it = m_ip_other.begin();
        std::advance( ip2_it, indices[i] );

        if (found_epipolar){
          Vector2 ip2_org_coord = Vector2( ip2_it->x, ip2_it->y );
          double  line_distance = m_matcher.distance_point_line( line_eq, ip2_org_coord );
          if ( line_distance < large_epipolar_threshold ) {
            if ( line_distance < small_epipolar_threshold )
              kept_indices.push_back( std::pair<float,int>( distances[i], indices[i] ) );
            else // In between thresholds
              kept_indices.push_back( std::pair<float,int>( distances[i], -1 ) );
          }
          else {
            Vector2 ip1_coord( ip->x, ip->y );
            //double normDist = norm_2(ip1_coord - ip2_org_coord);
            //vw_out() << "Discarding match between " << ip1_coord << " and " << ip2_org_coord
            //        << " because distance is " << line_distance << " and threshold is "
            //        << m_matcher.m_epipolar_threshold << " norm dist = " << normDist<<"\n";
          }
        }
      } // End loop for match prunining

        // If we only found one match or the first descriptor match is much better than the second
      if ( ( (kept_indices.size() > 2)     &&
             (kept_indices[0].second >= 0) &&
             (kept_indices[0].first < m_matcher.m_uniqueness_threshold * kept_indices[1].first) )
           || (kept_indices.size() == 1) ){
        *m_output++ = kept_indices[0].second; // Return the first of the matches we found
        //vw_out() << "Kept distance: " << kept_indices[0].first << std::endl;
      } else { // No matches or no clear winner
        *m_output++ = (size_t)(-1); // Failed to find a match, return a flag!
      }
    } // End loop through IP
  } // End function operator()

}; // End class EpipolarLineMatchTask -------------------

void EpipolarLinePointMatcher::operator()( ip::InterestPointList const& ip1,
                                           ip::InterestPointList const& ip2,
                                           DetectIpMethod  ip_detect_method,
                                           camera::CameraModel        * cam1,
                                           camera::CameraModel        * cam2,
                                           std::vector<size_t>        & output_indices ) const {
  typedef ip::InterestPointList::const_iterator IPListIter;

  Timer total_time("Total elapsed time", DebugMessage, "interest_point");
  size_t ip1_size = ip1.size(), ip2_size = ip2.size();

  output_indices.clear();
  if (!ip1_size || !ip2_size) {
    vw_out(InfoMessage,"interest_point") << "KD-Tree: no points to match, exiting\n";
    return;
  }

  // Build the output indices
  output_indices.resize(ip1_size);

  // Set up FLANNTree objects of all the different types we may need.
  math::FLANNTree<float>         kd_float;
  math::FLANNTree<unsigned char> kd_uchar;

  Matrix<float>         ip2_matrix_float;
  Matrix<unsigned char> ip2_matrix_uchar;

  // Pack the IP descriptors into a matrix and feed it to the chosen FLANNTree object
  const bool use_uchar_FLANN = (ip_detect_method == DETECT_IP_METHOD_ORB);
  if (use_uchar_FLANN) {
    ip_list_to_matrix(ip2, ip2_matrix_uchar);
    kd_uchar.load_match_data( ip2_matrix_uchar, vw::math::FLANN_DistType_Hamming );
  }else {
    ip_list_to_matrix(ip2, ip2_matrix_float);
    kd_float.load_match_data( ip2_matrix_float,  vw::math::FLANN_DistType_L2 );
  }

  vw_out(InfoMessage,"interest_point") << "FLANN-Tree created. Searching...\n";

  FifoWorkQueue matching_queue; // Create a thread pool object
  Mutex camera_mutex;

  // Jobs set to 2x the number of cores. This is just incase all jobs are not equal.
  // The total number of interest points will be divided up among the jobs.
  size_t number_of_jobs = vw_settings().default_num_threads() * 2;
#if __APPLE__
  // Fix due to OpenBLAS crashing and/or giving different results
  // each time. May need to be revisited.
  number_of_jobs = std::min(int(vw_settings().default_num_threads()), 1);
  vw_out() << "\t    Using " << number_of_jobs << " thread(s) for matching.\n";
#endif
    
  // Robustness fix
  if (ip1_size < number_of_jobs)
    number_of_jobs = ip1_size;

  // Get input and output iterators
  IPListIter start_it = ip1.begin();
  std::vector<size_t>::iterator output_it = output_indices.begin();

  for ( size_t i = 0; i < number_of_jobs - 1; i++ ) { // For each job...
    // Update iterators and launch the job.
    IPListIter end_it = start_it;
    std::advance( end_it, ip1_size / number_of_jobs );
    boost::shared_ptr<Task>
      match_task( new EpipolarLineMatchTask( m_single_threaded_camera,
                                             use_uchar_FLANN, kd_float, kd_uchar,
                                             start_it, end_it,
                                             ip2, cam1, cam2, *this,
                                             camera_mutex, output_it ) );
    matching_queue.add_task( match_task );
    start_it = end_it;
    std::advance( output_it, ip1_size / number_of_jobs );
  }
  boost::shared_ptr<Task>
    match_task( new EpipolarLineMatchTask( m_single_threaded_camera,
                                           use_uchar_FLANN, kd_float, kd_uchar,
                                           start_it, ip1.end(),
                                           ip2, cam1, cam2, *this,
                                           camera_mutex, output_it ) );
  matching_queue.add_task( match_task );
  matching_queue.join_all(); // Wait for all the jobs to finish.
}

// End class EpipolarLinePointMatcher
//---------------------------------------------------------------------------------------

void check_homography_matrix(Matrix<double>       const& H,
                             std::vector<Vector3> const& left_points,
                             std::vector<Vector3> const& right_points,
                             std::vector<size_t>  const& indices
                             ){

  // Sanity checks. If these fail, most likely the two images are too different
  // for stereo to succeed.
  if ( indices.size() < std::min( right_points.size(), left_points.size() )/2 ){
    vw_out(WarningMessage) << "InterestPointMatching: The number of inliers is less "
                           << "than 1/2 of the number of points. The inputs may be invalid.\n";
  }

  double det = fabs(H(0, 0)*H(1, 1) - H(0, 1)*H(1, 0));
  if (det <= 0.1 || det >= 10.0){
    vw_out(WarningMessage) << "InterestPointMatching: The determinant of the 2x2 submatrix "
                           << "of the homography matrix " << H << " is " << det
                           << ". There could be a large scale discrepancy among the input images "
                           << "or the inputs may be an invalid stereo pair.\n";
  }

}

// Find a rough homography that maps right to left using the camera
// and datum information.  More precisely, take a set of pixels in
// the left camera image, project them onto the ground and back
// project them into the right camera image. Then to the
// reverse. This will help find a rough correspondence between
// the pixels in the two camera images.
Matrix<double>
rough_homography_fit(camera::CameraModel* cam1,
                     camera::CameraModel* cam2,
                     BBox2i const& box1, BBox2i const& box2,
                     cartography::Datum const& datum) {
    
  // Bounce several points off the datum and fit an affine.
  std::vector<Vector3> left_points, right_points;
  int num = 100;
  left_points.reserve(2*num*num);
  right_points.reserve(2*num*num);

  // Report progress, as this may be slow
  TerminalProgressCallback tpc("", "\tRough homography--> ");
  tpc.report_progress(0);
  double inc_amount = 1.0 / double(num) / double(num);
    
  for (int i = 0; i < num; i++ ) {
    for ( int j = 0; j < num; j++ ) {
      try {
        Vector2 l( double(box1.width()  - 1) * i / (num-1.0),
                   double(box1.height() - 1) * j / (num-1.0) );

        Vector3 intersection = cartography::datum_intersection( datum, cam1, l );
        if ( intersection == Vector3() )
          continue;

        Vector2 r = cam2->point_to_pixel( intersection );

        if ( box2.contains( r ) ){
          left_points.push_back(  Vector3(l[0],l[1],1) );
          right_points.push_back( Vector3(r[0],r[1],1) );
        }
      }
      catch (...) {}

      try {
        Vector2 r(double(box2.width()  - 1) * i / (num-1.0),
                  double(box2.height() - 1) * j / (num-1.0) );

        Vector3 intersection = cartography::datum_intersection( datum, cam2, r );
        if ( intersection == Vector3() )
          continue;

        Vector2 l = cam1->point_to_pixel( intersection );

        if ( box1.contains( l ) ) {
          left_points.push_back( Vector3(l[0],l[1],1) );
          right_points.push_back( Vector3(r[0],r[1],1) );
        }
      }
      catch (...) {}
      tpc.report_incremental_progress( inc_amount );
    }
  }
  tpc.report_finished();
    
  if (left_points.empty() || right_points.empty())
    vw_throw( ArgumentErr() << "InterestPointMatching: rough_homography_fit failed to generate points! Examine your images, or consider using the options --skip-rough-homography and --no-datum.\n" );

  double thresh_factor = stereo_settings().ip_inlier_factor; // 1/15 by default
  typedef math::HomographyFittingFunctor hfit_func;
  math::RandomSampleConsensus<hfit_func, math::InterestPointErrorMetric>
    ransac( hfit_func(), math::InterestPointErrorMetric(),
            stereo_settings().ip_num_ransac_iterations,
            norm_2(Vector2(box1.width(),box1.height())) * (1.5*thresh_factor), // inlier threshold
            left_points.size()/2 // min output inliers
            );
  Matrix<double> H = ransac( right_points, left_points );
  std::vector<size_t> indices = ransac.inlier_indices(H, right_points, left_points);
  check_homography_matrix(H, left_points, right_points, indices);

  vw_out() << "Projected " << left_points.size()
           << " rays for rough homography.\n";
  vw_out() << "Number of inliers: " << indices.size() << ".\n";
    
  return H;
}

Vector2i homography_rectification(bool adjust_left_image_size,
                                  Vector2i const& left_size,
                                  Vector2i const& right_size,
                                  std::vector<ip::InterestPoint> const& left_ip,
                                  std::vector<ip::InterestPoint> const& right_ip,
                                  vw::Matrix<double>& left_matrix,
                                  vw::Matrix<double>& right_matrix) {
  // Reformat the interest points for RANSAC
  std::vector<Vector3>  right_copy = iplist_to_vectorlist(right_ip),
    left_copy  = iplist_to_vectorlist(left_ip);

  double thresh_factor = stereo_settings().ip_inlier_factor; // 1/15 by default
    
  // Use RANSAC to determine a good homography transform between the images
  math::RandomSampleConsensus<math::HomographyFittingFunctor, math::InterestPointErrorMetric>
    ransac( math::HomographyFittingFunctor(),
            math::InterestPointErrorMetric(),
            stereo_settings().ip_num_ransac_iterations,
            norm_2(Vector2(left_size.x(),left_size.y())) * (1.5*thresh_factor), // inlier thresh
            left_copy.size()*2/3 // min output inliers
            );
  Matrix<double> H = ransac(right_copy, left_copy);
  std::vector<size_t> indices = ransac.inlier_indices(H, right_copy, left_copy);
  check_homography_matrix(H, left_copy, right_copy, indices);

  // Set right to a homography that has been refined just to our inliers
  left_matrix  = math::identity_matrix<3>();
  right_matrix = math::HomographyFittingFunctor()(right_copy, left_copy, H);

  // Work out the ideal render size
  BBox2i output_bbox, right_bbox;
  output_bbox.grow( Vector2i(0,0) );
  output_bbox.grow( Vector2i(left_size.x(),0) );
  output_bbox.grow( Vector2i(0,left_size.y()) );
  output_bbox.grow( left_size );

  if (adjust_left_image_size){
    // Crop the left and right images to the shared region. This is
    // done for efficiency.  It may not be always desirable though,
    // as in this case we lose the one-to-one correspondence between
    // original input left image pixels and output disparity/point
    // cloud pixels.
    Vector3 temp = right_matrix*Vector3(0,0,1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    temp = right_matrix*Vector3(right_size.x(),0,1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    temp = right_matrix*Vector3(0,right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );
    temp = right_matrix*Vector3(right_size.x(),right_size.y(),1);
    temp /= temp.z();
    right_bbox.grow( subvector(temp,0,2) );

    output_bbox.crop( right_bbox );

    //  Move the ideal render size to be aligned up with origin
    left_matrix (0,2) -= output_bbox.min().x();
    right_matrix(0,2) -= output_bbox.min().x();
    left_matrix (1,2) -= output_bbox.min().y();
    right_matrix(1,2) -= output_bbox.min().y();
  }

  return Vector2i(output_bbox.width(), output_bbox.height());
}

/// Remove points in/out of a bounding box depending on "remove_outside".
/// - Returns the number of points removed.
size_t remove_ip_bbox(vw::BBox2i const& roi, vw::ip::InterestPointList & ip_list,
                      bool remove_outside){
  // Loop through all the points
  size_t num_removed = 0;
  vw::ip::InterestPointList::iterator ip;
  for (ip = ip_list.begin(); ip != ip_list.end(); ++ip) {
    
    if (roi.contains(vw::Vector2i(ip->ix,ip->iy)) xor remove_outside) {
      ip = ip_list.erase(ip);
      ++num_removed;
      --ip;
    }
  }
  return num_removed;
} // End function remove_ip_bbox
  
void side_ip_filtering(vw::ip::InterestPointList& ip1, 
                       vw::ip::InterestPointList& ip2,  
                       vw::BBox2i const& bbox1, vw::BBox2i const& bbox2) {
      
  // Filter out IP from the opposite sides of the two images.
  // - Would be better to just pass an ROI into the IP detector!
  if (stereo_settings().ip_edge_buffer_percent <= 0)
    return;
      
  // Figure out removal bboxes
  double percent  = static_cast<double>(stereo_settings().ip_edge_buffer_percent)/100.0;
  int   width_left  = floor(static_cast<double>(bbox1.width()) * percent);
  int   width_right = floor(static_cast<double>(bbox2.width()) * percent);
  BBox2 bbox_left (width_left, 0, bbox1.width()-width_left,  bbox1.height());
  BBox2 bbox_right(0,          0, bbox2.width()-width_right, bbox2.height());
  bool  remove_outside = true;
    
  // Remove the points
  size_t num_removed_left  = remove_ip_bbox(bbox_left,  ip1, remove_outside);
  size_t num_removed_right = remove_ip_bbox(bbox_right, ip2, remove_outside);
  vw_out() << "Removed: " << num_removed_left << " points from the left side of the left image and "
           << num_removed_right << " points from the right side of the right image.\n";
} // End side IP filtering
  
bool tri_ip_filtering( std::vector<ip::InterestPoint> const& matched_ip1,
                  std::vector<ip::InterestPoint> const& matched_ip2,
                  vw::camera::CameraModel* cam1,
                  vw::camera::CameraModel* cam2,
                  std::list<size_t>& valid_indices ) {
  typedef std::vector<double> ArrayT;
  ArrayT error_samples( valid_indices.size() );

  // Create the 'error' samples. Which are triangulation error and distance to sphere.
  double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);

  stereo::StereoModel model( cam1, cam2, stereo_settings().use_least_squares, angle_tol );
  size_t count = 0;
  const double HIGH_ERROR = 9999999;
  BOOST_FOREACH( size_t i, valid_indices ) {
    model( Vector2(matched_ip1[i].x, matched_ip1[i].y),
           Vector2(matched_ip2[i].x, matched_ip2[i].y), error_samples[count] );
    // The call returns exactly zero error to indicate a failed ray intersection
    //  so replace it in those cases with a very high error
    if (error_samples[count] == 0)
      error_samples[count] = HIGH_ERROR;
    count++;
  }
  VW_ASSERT( count == valid_indices.size(),
             vw::MathErr() << "tri_ip_filtering: Programmer error. Count indices not aligned." );

  typedef std::vector<std::pair<Vector<double>, Vector<double> > > ClusterT;
  const size_t NUM_CLUSTERS = 2;
  ClusterT error_clusters = gaussian_clustering<ArrayT>(error_samples.begin(), error_samples.end(), NUM_CLUSTERS);

  // The best triangulation error is the one that has the smallest
  // standard deviations. They are focused on the tight pack of
  // inliers. Bring the smaller std-dev cluster to the front as it
  // is what we are interested in.OB
  if ( (error_clusters.front().second[0] > error_clusters.back().second[0]       ) &&
       (error_clusters.back().second[0] != std::numeric_limits<double>::epsilon())   )
    std::swap( error_clusters[0], error_clusters[1] );

  // Determine if we just wrote nothing but outliers (the variance
  // on triangulation is too high).
  if ( error_clusters.front().second[0] > 1e6 ) {

    vw_out() << "\t    Unable to find inlier cluster, keeping best 70% of points.\n";

    // Instead of failing, take our best guess at which points are inliers
    // - Sort the error, then find an approximate percentile.
    const double CUTOFF_PERCENTILE = 0.7;
    std::vector<double> sorted_error(error_samples);
    std::sort(sorted_error.begin(), sorted_error.end());
    size_t last_good_index = static_cast<double>(sorted_error.size()) * CUTOFF_PERCENTILE;
    double cutoff_value = 0;
    if (last_good_index < sorted_error.size())
      cutoff_value = sorted_error[last_good_index];
    // The maximum triangulation error still applies!
    if ((asp::stereo_settings().ip_triangulation_max_error > 0) && 
        (cutoff_value > asp::stereo_settings().ip_triangulation_max_error))
      cutoff_value = asp::stereo_settings().ip_triangulation_max_error;

    // Treat all points below the new cutoff_value as inliers
    std::list<size_t> filtered_indices;
    size_t c=0;
    for ( std::list<size_t>::iterator i = valid_indices.begin(); i != valid_indices.end(); i++ )
      {
        if (error_samples[c] < cutoff_value)
          filtered_indices.push_back(*i);
        ++c;
      }
    valid_indices = filtered_indices;
    return (!valid_indices.empty());
  } // End of "nothing but outliers" case

  vw_out() << "\t    Inlier cluster:\n"
           << "\t      Triangulation error: " << error_clusters.front().first[0]
           << " +- " << sqrt( error_clusters.front().second[0] ) << " meters\n";

  // Record indices of points that match our clustering result
  const double escalar1 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.front().second[0] ); // outside exp of normal eq
  const double escalar2 = 1.0 / sqrt( 2.0 * M_PI * error_clusters.back().second[0] );
  const double escalar3 = 1.0 / (2 * error_clusters.front().second[0] ); // inside exp of normal eq
  const double escalar4 = 1.0 / (2 * error_clusters.back().second[0] );
  size_t error_idx        = 0;
  size_t prior_valid_size = valid_indices.size();
  size_t outlier_count    = 0;
  for ( std::list<size_t>::iterator i = valid_indices.begin(); i != valid_indices.end(); i++ ) {
    double err_diff_front = error_samples[error_idx]-error_clusters.front().first[0];
    double err_diff_back  = error_samples[error_idx]-error_clusters.back ().first[0];

    if ( !((escalar1 * exp( (-err_diff_front * err_diff_front) * escalar3 ) ) >
           (escalar2 * exp( (-err_diff_back  * err_diff_back ) * escalar4 ) ) ||
           error_samples[error_idx] < error_clusters.front().first[0]          ) ||
         ( (asp::stereo_settings().ip_triangulation_max_error > 0) &&
           (error_samples[error_idx] > asp::stereo_settings().ip_triangulation_max_error) ) ) {
      // It's an outlier!
      //vw_out() << "Removing error_samples["<< error_idx <<"] = " << error_samples[error_idx] << std::endl;
      i = valid_indices.erase(i);
      i--; // For loop is going to increment this back up
      ++outlier_count;
    }
    error_idx++;
  }
  VW_ASSERT( prior_valid_size == error_idx,
             vw::MathErr() << "tri_ip_filtering: Programmer error. Indices don't seem to be aligned." );

  vw_out() << "\t      Removed " << outlier_count << " points in triangulation filtering.\n";
  return (!valid_indices.empty());
}

bool stddev_ip_filtering( std::vector<vw::ip::InterestPoint> const& ip1,
                          std::vector<vw::ip::InterestPoint> const& ip2,
                          std::list<size_t>& valid_indices ) {
  const int NUM_STD_FILTER = 4;
  // 4 stddev filtering. Deletes any disparity measurement that is 4
  // stddev away from the measurements of it's local neighbors. We
  // kill off worse offender one at a time until everyone is compliant.
  bool deleted_something;
  size_t num_deleted = 0;
  do {
    deleted_something = false;
    Matrix<float> locations1( valid_indices.size(), 2 );
    size_t count = 0;
    std::vector<size_t > reverse_lookup  ( valid_indices.size() );
    std::vector<Vector2> disparity_vector( valid_indices.size() );
    BOOST_FOREACH( size_t index, valid_indices ) {
      locations1( count, 0 ) = ip1[index].x;
      locations1( count, 1 ) = ip1[index].y;
      reverse_lookup  [ count ] = index;
      disparity_vector[ count ] = Vector2(ip2[index].x,ip2[index].y) -
        Vector2(ip1[index].x,ip1[index].y);
      count++;
    }
    math::FLANNTree<float> tree1;
    tree1.load_match_data(locations1, vw::math::FLANN_DistType_L2);

    std::pair<double,size_t> worse_index;
    worse_index.first = 0;
    for ( size_t i = 0; i < valid_indices.size(); i++ ) {
      Vector<int   > indices;
      Vector<double> distance;
      const int NUM_INDICES_TO_GET = 11;
      tree1.knn_search(select_row( locations1, i ),
                       indices, distance, NUM_INDICES_TO_GET);

      // Bugfix: If there are too few inputs, in rare occasions
      // some of the outputs are invalid. Not always. Could not
      // figure this out in reasonable time, the logic is somewhere
      // deep inside FLANN. Just discard the bad results.
      std::vector<int> good_indices;
      for (size_t j = 0; j < indices.size(); j++) {
        if ((indices[j] < 0) || (indices[j] >= (int)disparity_vector.size()))
          continue;
        good_indices.push_back(indices[j]);
      }

      // Make an average of the disparities around us and not our own measurement
      Vector2 sum;
      for ( size_t j = 1; j < good_indices.size(); j++ ) {
        sum += disparity_vector[ good_indices[j] ];
      }
      sum = normalize( sum );

      // Project all disparities along the new gradient
      Vector<double> projections( good_indices.size() );
      for ( size_t j = 0; j < good_indices.size(); j++ ) {
        projections[j] = dot_prod( disparity_vector[good_indices[j]], sum );
      }
      double mean   = 0;
      double stddev = 0;
      for ( size_t j = 1; j < good_indices.size(); j++ ) {
        mean += projections[j];
        stddev += projections[j]*projections[j];
      }
      mean /= good_indices.size() - 1;
      stddev = sqrt( stddev / ( good_indices.size() - 1 ) - mean*mean );

      double std_distance = fabs(projections[0]-mean)/stddev;
      if ( std_distance > worse_index.first ) {
        worse_index.first = std_distance;
        worse_index.second = i;
      }
    } // End loop through valid indices
    if ( worse_index.first > NUM_STD_FILTER ) {
      std::list<size_t>::iterator it = valid_indices.begin();
      std::advance( it, worse_index.second );
      valid_indices.erase( it );
      deleted_something = true;
      ++num_deleted;
    }
    // If we ended up deleting everything, just quit here and return 0.
    if (valid_indices.empty())
      return 0;
  } while( deleted_something );

  vw_out() << "\t      Removed " << num_deleted << " points in stddev filtering.\n";
  return valid_indices.size();
}

size_t filter_ip_by_lonlat_and_elevation(vw::TransformPtr         tx_left,
                                         vw::TransformPtr         tx_right,
                                         vw::camera::CameraModel* left_camera_model,
                                         vw::camera::CameraModel* right_camera_model,
                                         vw::cartography::Datum        const& datum,
                                         std::vector<vw::ip::InterestPoint> const& ip1_in,
                                         std::vector<vw::ip::InterestPoint> const& ip2_in,
                                         vw::Vector2 const & elevation_limit,
                                         vw::BBox2   const & lon_lat_limit,
                                         std::vector<vw::ip::InterestPoint> & ip1_out,
                                         std::vector<vw::ip::InterestPoint> & ip2_out){

  // Handle case where the elevation or lonlat range are not set
  const size_t num_ip = ip1_in.size();                              
  if (elevation_limit[1] <= elevation_limit[0] && lon_lat_limit.empty()) {
    ip1_out = ip1_in;
    ip2_out = ip2_in;
    return num_ip;
  }

  if (elevation_limit[0] < elevation_limit[1]) 
    vw_out() << "\t    * Applying elevation restriction. Height range: " << elevation_limit[0]
             << " to " << elevation_limit[1] << ".\n";

  if (!lon_lat_limit.empty()) 
    vw_out() << "\t    * Applying lon-lat restriction: " << lon_lat_limit.min()
             << " to " << lon_lat_limit.max() << ".\n";
    
  // Init output vectors
  ip1_out.clear();
  ip2_out.clear();
  ip1_out.reserve(num_ip);
  ip2_out.reserve(num_ip);

  // Set up stereo model
  double angle_tolerance = vw::stereo::StereoModel::robust_1_minus_cos(
                                                                       stereo_settings().min_triangulation_angle*M_PI/180);
  vw::stereo::StereoModel model(left_camera_model, right_camera_model, 
                                stereo_settings().use_least_squares, angle_tolerance );
    
  // This function can be called with both unaligned and aligned interest points
  bool aligned_ip = (tx_left.get() != NULL && tx_right != NULL);

  // For each interest point, compute the height and only keep it if the height falls within
  // the specified range.
  for (size_t i = 0; i < num_ip; ++i) {

    // We must not both apply a transform and a scale at the same time
    // as these are meant to do the same thing in different circumstances.
    Vector2 p1 = Vector2(ip1_in[i].x, ip1_in[i].y);
    Vector2 p2 = Vector2(ip2_in[i].x, ip2_in[i].y);

    if (aligned_ip) {
      // Unalign
      p1 = tx_left->reverse (p1);
      p2 = tx_right->reverse(p2);
    }
      
    // Triangulate
    double err = -1.0;
    Vector3 xyz(0.0, 0.0, 0.0);
    try {
      xyz = model(p1, p2, err);
    } catch(...) {
      xyz = Vector3();
    }

    if (err <= 0.0 || xyz == Vector3()) {
      // Triangulation failed
      continue;
    }
      
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    if ( (elevation_limit[0] < elevation_limit[1]) && 
         ( (llh[2] < elevation_limit[0]) || (llh[2] > elevation_limit[1]) ) ) {
      // vw_out() << "Removing IP diff: " << p2 - p1 << " with llh " << llh << std::endl;
      continue;
    }
      
    Vector2 lon_lat = subvector(llh, 0, 2);
    if ( (!lon_lat_limit.empty()) && (!lon_lat_limit.contains(lon_lat)) ) {
      continue; 
    }
      
    // vw_out() << "Keeping IP diff: " << p2 - p1 << " with llh " << llh << std::endl;
    ip1_out.push_back(ip1_in[i]);
    ip2_out.push_back(ip2_in[i]);
  }
  vw_out() << "\t    * Removed " << ip1_in.size() - ip1_out.size()
           << " ip using elevation/lonlat filtering.\n";
    
  return ip1_out.size();
} // End filter_ip_by_elevation
  
  // Filter ip by triangulation error, reprojection error, and height range.
  // This assumes the interest points are for the original images, without alignment
  // or mapprojection.
  // TODO(oalexan1): Add here the ability to reverse the alignment
void filter_ip_using_cameras(std::vector<vw::ip::InterestPoint> & ip1,
                             std::vector<vw::ip::InterestPoint> & ip2,
                             vw::camera::CameraModel const * cam1,
                             vw::camera::CameraModel const * cam2,
                             vw::cartography::Datum  const & datum,
                             double pct, double factor) {

  std::vector<double> errors(ip1.size()), heights(ip1.size());
    
  // Compute the triangulation errors
  double angle_tol = vw::stereo::StereoModel
    ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
  stereo::StereoModel model(cam1, cam2, stereo_settings().use_least_squares, angle_tol);
  double HIGH_ERROR = std::numeric_limits<double>::max();
  for (size_t i = 0; i < ip1.size(); i++) {
    Vector3 xyz;
    try {
      xyz = model(Vector2(ip1[i].x, ip1[i].y), Vector2(ip2[i].x, ip2[i].y),
                  errors[i]);
    } catch(...) {
      xyz = Vector3();
    }
      
    // The call returns the zero error and zero xyz to indicate a
    // failed ray intersection so replace it in those cases with a
    // very high error.
    if (errors[i] == 0 || xyz == Vector3()) {
      errors[i]  = HIGH_ERROR;
      heights[i] = HIGH_ERROR;
      continue;
    }

    // Find the height above datum
    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    heights[i] = llh[2];
  }

  // Put the valid heights in a vector
  std::vector<double> vals;
  vals.reserve(ip1.size());
  vals.clear();
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR) continue;
    vals.push_back(heights[i]);
  }

  // Find the outlier brackets
  double pct_fraction = 1.0 - pct/100.0;
  double b = -1.0, e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction, factor, b, e);
    
  // Apply the outlier threshold
  int count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR) continue;
    if (heights[i] < b || heights[i] > e) {
      heights[i] = HIGH_ERROR;
      count++;
    }
  }
  vw_out() << "Number (and fraction) of removed outliers by the height check: "
           << count << " (" << double(count)/ip1.size() << ").\n";

  // Find the valid errors. Make use of the fact that we already filtered by height.
  vals.clear();
  for (size_t i = 0; i < ip1.size(); i++) {
    if (errors[i] >= HIGH_ERROR || heights[i] >= HIGH_ERROR) // already invalid
      continue;
    vals.push_back(errors[i]);
  }

  // Find the outlier brackets. Since the triangulation errors, unlike
  // the heights, are usually rather uniform, adjust pct from 95 to
  // 75.
  double pct2 = (75.0/95.0) * pct;
  double pct_fraction2 = 1.0 - pct2/100.0;
  // Show some lenience below as due to jitter some errors could be somewhat bigger
  double factor2 = 2.0 * factor;
  b = -1.0; e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction2, factor2, b, e);

  // Apply the outlier threshold
  count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR || errors[i] >= HIGH_ERROR) continue; // already invalid
    // We will ignore b, as the triangulation errors are non-negative.
    if (errors[i] > e) {
      errors[i] = HIGH_ERROR;
      count++;
    }
  }
    
  vw_out() << "Number (and fraction) of removed outliers by the triangulation error check: "
           << count << " (" << double(count)/ip1.size() << ").\n";
    
  // Copy the outliers in place
  count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (errors[i] >= HIGH_ERROR || heights[i] >= HIGH_ERROR) 
      continue;

    ip1[count] = ip1[i];
    ip2[count] = ip2[i];
    count++;
  }

  ip1.resize(count);
  ip2.resize(count);
}
  
// Outlier removal based on the disparity of interest points.
// Points with x or y disparity not within the 100-'pct' to 'pct'
// percentile interval expanded by 'factor' will be removed as
// outliers. Overwrite the ip in place.
void filter_ip_by_disparity(double pct, // for example, 90.0
                            double factor, // for example, 3.0
                            std::vector<vw::ip::InterestPoint> & left_ip,
                            std::vector<vw::ip::InterestPoint> & right_ip){

  double pct_fraction = 1.0 - pct/100.0;
  double bx, ex, by, ey;
  std::vector<double> dispx, dispy;
  for (size_t it = 0; it < left_ip.size(); it++) {
    dispx.push_back(right_ip[it].x - left_ip[it].x);
    dispy.push_back(right_ip[it].y - left_ip[it].y);
  }
  vw::math::find_outlier_brackets(dispx, pct_fraction, factor, bx, ex);
  vw::math::find_outlier_brackets(dispy, pct_fraction, factor, by, ey);
    
  //vw_out() << "Outlier statistics by disparity in x: b = " << bx << ", e = " << ex << ".\n";
  //vw_out() << "Outlier statistics by disparity in y: b = " << by << ", e = " << ey << ".\n";
    
  // Remove the bad ip 
  size_t good_it = 0;
  for (size_t it = 0; it < left_ip.size(); it++) {
    if (dispx[it] < bx || dispx[it] > ex) continue;
    if (dispy[it] < by || dispy[it] > ey) continue;
    left_ip [good_it] = left_ip[it];
    right_ip[good_it] = right_ip[it];
    good_it++;
  }
  vw_out() << "Removed " << left_ip.size() - good_it
           << " outliers based on percentiles of differences of interest "
           << "points with --outlier-removal-params.\n";
  
  left_ip.resize(good_it);
  right_ip.resize(good_it);

  return;
}

size_t filter_ip_homog(std::vector<ip::InterestPoint> const& ip1_in,
                       std::vector<ip::InterestPoint> const& ip2_in,
                       std::vector<ip::InterestPoint>      & ip1_out,
                       std::vector<ip::InterestPoint>      & ip2_out,
                       int inlier_threshold) {

  std::vector<size_t> indices;
  try {
    
    std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(ip1_in),
      ransac_ip2 = iplist_to_vectorlist(ip2_in);

    vw_out() << "\t    Inlier threshold:                     " << inlier_threshold << "\n";
    vw_out() << "\t    RANSAC iterations:                    "
             << stereo_settings().ip_num_ransac_iterations << "\n";
    typedef math::RandomSampleConsensus<math::HomographyFittingFunctor,
      math::InterestPointErrorMetric> RansacT;
    const int    MIN_NUM_OUTPUT_INLIERS = ransac_ip1.size()/2;
    RansacT ransac( math::HomographyFittingFunctor(),
                    math::InterestPointErrorMetric(),
                    stereo_settings().ip_num_ransac_iterations,
                    inlier_threshold,
                    MIN_NUM_OUTPUT_INLIERS, true);
    Matrix<double> H(ransac(ransac_ip2,ransac_ip1)); // 2 then 1 is used here for legacy reasons
    //vw_out() << "\t--> Homography: " << H << "\n";
    indices = ransac.inlier_indices(H,ransac_ip2,ransac_ip1);
  } catch (const math::RANSACErr& e ) {
    vw_out() << "RANSAC failed: " << e.what() << "\n";
    return false;
  }

  // Assemble the remaining interest points
  const size_t num_left = indices.size();
  std::vector<ip::InterestPoint> final_ip1, final_ip2;
  ip1_out.resize(num_left);
  ip2_out.resize(num_left);
  for (size_t i=0; i<num_left; ++i) {
    size_t index = indices[i];
    ip1_out[i] = ip1_in[index];
    ip2_out[i] = ip2_in[index];
  }

  return num_left;
}


// Filter IP using a given DEM and max height difference.  Assume that
// the interest points have alignment applied to them (either via a
// transform or from mapprojection).
void ip_filter_using_dem(std::string              const & ip_filter_using_dem,
                         vw::TransformPtr                 tx_left,
                         vw::TransformPtr                 tx_right,
                         boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                         boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                         std::vector<vw::ip::InterestPoint> & left_aligned_ip,
                         std::vector<vw::ip::InterestPoint> & right_aligned_ip) {

  vw_out() << "Filtering interest point matches with --ip-filter-using-dem.\n";
  
  std::string dem_file;
  double max_height_diff = -1.0;
  std::istringstream is(ip_filter_using_dem);
  if (!(is >> dem_file >> max_height_diff)) 
    vw_throw(ArgumentErr() << "Could not parse correctly option --ip-filter-using-dem.\n");

  if (max_height_diff <= 0.0) 
    vw_throw(ArgumentErr() << "Positive height diff value expected in --ip-filter-using-dem.\n");

  // Read the DEM and supporting data
  vw::cartography::GeoReference dem_georef;
  DiskImageView<float> dem_disk_image(dem_file);
  ImageViewRef<PixelMask<float>> dem;
  boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResourcePtr(dem_file));
  if (dem_rsrc->has_nodata_read())
    dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
  else
    dem = pixel_cast<PixelMask<float>>(dem_disk_image); // all pixels are valid

  bool has_georef = vw::cartography::read_georeference(dem_georef, dem_file);
  if (!has_georef)
    vw_throw(ArgumentErr() << "There is no georeference information in: "
             << dem_file << ".\n" );
  
  // An invalid pixel value used for edge extension
  PixelMask<float> nodata_pix(0); nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> nodata_ext(nodata_pix); 
  
  // Set up for interpolation. Out-of-range pixels are declared to be invalid.
  ImageViewRef<PixelMask<float>> interp_dem
    = interpolate(dem, BilinearInterpolation(), nodata_ext);

  // Set up the stereo model for doing triangulation
  double angle_tol = vw::stereo::StereoModel
    ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
  stereo::StereoModel model(left_camera_model.get(), right_camera_model.get(),
                            stereo_settings().use_least_squares, angle_tol);

  std::set<int> invalid_indices;
  for (size_t it = 0; it < left_aligned_ip.size(); it++) {

    // Undo the alignment or mapprojection
    Vector2 left_pix  = tx_left->reverse (Vector2(left_aligned_ip[it].x,  left_aligned_ip[it].y));
    Vector2 right_pix = tx_right->reverse(Vector2(right_aligned_ip[it].x, right_aligned_ip[it].y));

    // Triangulate
    double err = -1.0;
    Vector3 xyz(0.0, 0.0, 0.0);
    try {
      xyz = model(left_pix, right_pix, err);
    } catch(...) {
      xyz = Vector3();
    }

    if (err <= 0.0 || xyz == Vector3()) {
      invalid_indices.insert(it);
      continue;
    }

    Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);

    // This was tested to give correct results with the DEM
    // having its longitude in both [-180, 0] and [180, 360].
    Vector2 pix = dem_georef.lonlat_to_pixel(Vector2(llh[0], llh[1]));

    PixelMask<float> dem_val = interp_dem(pix.x(), pix.y());

    if (!is_valid(dem_val) || std::abs(dem_val.child() - llh[2]) > max_height_diff)
      invalid_indices.insert(it);
  }

  int num_total   = left_aligned_ip.size();
  int num_invalid = invalid_indices.size();
  int num_valid   = num_total - num_invalid;
  vw_out() << "Removed " << num_invalid << " interest points out of " << num_total << " ("
           << 100.0 * double(num_invalid) / num_total << " pct).\n";

  // Copy back only the valid pixels in-place
  int good_it = 0;
  for (int it = 0; it < num_total; it++) {
    if (invalid_indices.find(it) != invalid_indices.end())
      continue;
      
    left_aligned_ip[good_it]  = left_aligned_ip[it];
    right_aligned_ip[good_it] = right_aligned_ip[it];
    good_it++;
  }

  if (good_it != num_valid)
    vw_throw(ArgumentErr() << "Book-keeping failure in ip filtering using DEM.\n");
    
  left_aligned_ip.resize(num_valid);
  right_aligned_ip.resize(num_valid);

  return;
}

// Estimate the search range by finding the median disparity and
// creating a box of given dimensions around it. This assumes aligned
// interest points. Note that this box may be an overestimate,
// so it should be intersected with a previously existing box.
vw::BBox2 search_range_using_spread(double max_disp_spread,
                                    std::vector<vw::ip::InterestPoint> const& left_ip,
                                    std::vector<vw::ip::InterestPoint> const& right_ip) {
    std::vector<double> dx, dy;
  
    for (size_t i = 0; i < left_ip.size(); i++) {
    double diffX = right_ip[i].x - left_ip[i].x;
    double diffY = right_ip[i].y - left_ip[i].y;
    dx.push_back(diffX);
    dy.push_back(diffY);
  }
  if (dx.empty())
    vw_throw(vw::ArgumentErr() << "No interest points left.");
  
  std::sort(dx.begin(), dx.end());
  std::sort(dy.begin(), dy.end());
  double mid_x = dx[dx.size()/2]; // median
  double mid_y = dy[dy.size()/2];
  
  double half = max_disp_spread / 2.0;
  vw::BBox2 spread_box(mid_x - half, mid_y - half, max_disp_spread, max_disp_spread);

  return spread_box;
}

// Create interest points from valid D_sub values and make them full scale
// (while still having potentially a global alignment applied to them).
void aligned_ip_from_D_sub(vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> const & sub_disp,
                           vw::Vector2                                   const & upsample_scale,
                           std::vector<vw::ip::InterestPoint>                  & left_ip, 
                           std::vector<vw::ip::InterestPoint>                  & right_ip) {

  left_ip.clear();
  right_ip.clear();
    
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);

      if (!is_valid(disp)) continue;
        
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      left_pix  = elem_prod(left_pix, upsample_scale);
      right_pix = elem_prod(right_pix, upsample_scale);

      vw::ip::InterestPoint left, right;
      left.x  = left_pix.x();   left.y = left_pix.y();
      right.x = right_pix.x(); right.y = right_pix.y();

      left_ip.push_back(left);
      right_ip.push_back(right);
    }
  }
}
  
double calc_ip_coverage_fraction(std::vector<ip::InterestPoint> const& ip,
                                 vw::Vector2i const& image_size, int tile_size,
                                 int min_ip_per_tile) {

  if (tile_size < 1)
    vw_throw(LogicErr() << "calc_ip_coverage_fraction: tile size is " << tile_size);

  // Generate a grid of ROIs covering the entire image
  BBox2i full_bbox(Vector2i(0,0), image_size);
  bool include_partials = false;
  std::vector<BBox2i> rois;
  rois = subdivide_bbox(full_bbox, tile_size, tile_size, include_partials);
  const size_t num_rois = rois.size();
  if (num_rois == 0)
    return 0; // Cannot have any coverage in the degenerate case!
    
  // Pack all IP into a list for speed
  std::list<Vector2i> ip_list;
  for (size_t i=0; i<ip.size(); ++i) {
    ip_list.push_back(Vector2i(ip[i].x, ip[i].y));
  }
    
  size_t num_filled_rois = 0;
  for (size_t i=0; i<num_rois; ++i) { // Loop through ROIs
    int ip_in_roi = 0;
      
    // Check if each point is in this ROI
    std::list<Vector2i>::iterator iter;
    for (iter=ip_list.begin(); iter!=ip_list.end(); ++iter) {
        
      // If the IP is in the ROI, remove it from the IP list so it
      // does not get searched again.
      if (rois[i].contains(*iter)) {
        iter = ip_list.erase(iter);
        ++ip_in_roi;
        --iter;
      }
    } // End IP loop
    if (ip_in_roi > min_ip_per_tile)
      ++num_filled_rois;
  }// End ROI loop

  return static_cast<double>(num_filled_rois) / static_cast<double>(num_rois);
}

// Do IP matching, return, the best translation+scale fitting functor.
vw::Matrix<double> translation_ip_matching(vw::ImageView<vw::PixelGray<float>> const& image1,
                                           vw::ImageView<vw::PixelGray<float>> const& image2,
                                           int ip_per_tile,
                                           std::string const  left_file_path,
                                           std::string const  right_file_path,
                                           double nodata1, double nodata2) {

  using namespace vw;

  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  detect_match_ip(matched_ip1, matched_ip2, image1,  image2, ip_per_tile,
                  left_file_path, right_file_path, nodata1, nodata2);
  
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);
  vw_out(DebugMessage,"asp") << "\t--> Removed "
                             << matched_ip1.size() - ransac_ip1.size()
                             << " duplicate matches.\n";

  Matrix<double> T;
  std::vector<size_t> indices;
  try {

    vw::math::RandomSampleConsensus<vw::math::TranslationScaleFittingFunctor, vw::math::InterestPointErrorMetric>
      ransac(vw::math::TranslationScaleFittingFunctor(),
             vw::math::InterestPointErrorMetric(),
             stereo_settings().ip_num_ransac_iterations,
             10, ransac_ip1.size()/2, true);
    T = ransac( ransac_ip2, ransac_ip1 );
    indices = ransac.inlier_indices(T, ransac_ip2, ransac_ip1 );
  } catch (...) {
    vw_out(WarningMessage,"console") << "Automatic Alignment Failed! Proceed with caution...\n";
    T = vw::math::identity_matrix<3>();
  }

  { // Keeping only inliers
    std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
    for ( size_t i = 0; i < indices.size(); i++ ) {
      inlier_ip1.push_back( matched_ip1[indices[i]] );
      inlier_ip2.push_back( matched_ip2[indices[i]] );
    }
    matched_ip1 = inlier_ip1;
    matched_ip2 = inlier_ip2;
  }

  return T;

}

/// The unwarped disparity file name
std::string unwarped_disp_file(std::string const& prefix, std::string const& left_image,
                               std::string const& right_image){
    
  std::string match_file = vw::ip::match_filename(prefix, left_image, right_image);

  std::string disp_file = boost::filesystem::path(match_file).replace_extension("").string();
  return disp_file + "-unaligned-D.tif";
}

}
