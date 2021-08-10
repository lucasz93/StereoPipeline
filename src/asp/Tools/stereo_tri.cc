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


/// \file stereo_tri.cc
///

#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/InterestPoint/InterestData.h>

#include <asp/Camera/RPCModel.h>
#include <asp/Tools/stereo.h>
#include <asp/Tools/jitter_adjust.h>
#include <asp/Tools/ccd_adjust.h>

// We must have the implementations of all sessions for triangulation
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/StereoSessionMapProj.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionSpot.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <xercesc/util/PlatformUtils.hpp>
#include <ctime>

using namespace vw;
using namespace asp;
using namespace std;

typedef typename StereoSession::tx_type TXT;

// A small function used in making a copy of the transform for map-projected images with
// a sanity check.
template<class T>
T make_transform_copy(T trans){
  vw::cartography::Map2CamTrans* t_ptr = dynamic_cast<vw::cartography::Map2CamTrans*>(trans.get());
  if (!t_ptr)
    vw_throw( NoImplErr() << "Need to support new map projection transform in stereo_tri.");
  return T(new vw::cartography::Map2CamTrans(*t_ptr));
}

/// The main class for taking in a set of disparities and returning a point cloud via joint triangulation.
template <class DisparityImageT, class StereoModelT>
class StereoTXAndErrorView : public ImageViewBase<StereoTXAndErrorView<DisparityImageT, StereoModelT> >
{
  vector<DisparityImageT> m_disparity_maps;
  vector<TXT>  m_transforms; // e.g., map-projection or homography to undo
  StereoModelT m_stereo_model;
  bool         m_is_map_projected;
  typedef typename DisparityImageT::pixel_type DPixelT;

public:

  typedef Vector6 pixel_type;
  typedef Vector6 result_type;
  typedef ProceduralPixelAccessor<StereoTXAndErrorView> pixel_accessor;

  /// Constructor
  StereoTXAndErrorView( vector<DisparityImageT> const& disparity_maps,
                        vector<TXT>             const& transforms,
                        StereoModelT            const& stereo_model,
                        bool is_map_projected) :
    m_disparity_maps(disparity_maps),
    m_transforms(transforms),
    m_stereo_model(stereo_model),
    m_is_map_projected(is_map_projected) {

    // Sanity check
    for (int p = 1; p < (int)m_disparity_maps.size(); p++){
      if (m_disparity_maps[0].cols() != m_disparity_maps[p].cols() ||
          m_disparity_maps[0].rows() != m_disparity_maps[p].rows()   )
        vw_throw( ArgumentErr() << "In multi-view triangulation, all disparities must have the same dimensions.\n" );
    }
  }

  inline int32 cols  () const { return m_disparity_maps[0].cols(); }
  inline int32 rows  () const { return m_disparity_maps[0].rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  /// Compute the 3D coordinate corresponding to a pixel location.
  /// - p is not actually used here, it should always be zero!
  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {
      
    // For each input image, de-warp the pixel in to the native camera coordinates
    int num_disp = m_disparity_maps.size();
    vector<Vector2> pixVec(num_disp + 1);
    pixVec[0] = m_transforms[0]->reverse(Vector2(i,j)); // De-warp "left" pixel
    for (int c = 0; c < num_disp; c++){
      Vector2 pix;
      DPixelT disp = m_disparity_maps[c](i,j,p); // Disparity value at this pixel
      if (is_valid(disp)) // De-warp the "right" pixel
        pix = m_transforms[c+1]->reverse( Vector2(i,j) + stereo::DispHelper(disp) );
      else // Insert flag values
        pix = Vector2(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());
      pixVec[c+1] = pix;
    }

    // Compute the location of the 3D point observed by each input pixel
    Vector3 errorVec;
    pixel_type result;
    subvector(result,0,3) = m_stereo_model(pixVec, errorVec);
    subvector(result,3,3) = errorVec;
    return result; // Contains location and error vector
  }

  typedef StereoTXAndErrorView<ImageViewRef<DPixelT>, StereoModelT> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return PreRasterHelper( bbox, m_transforms );
  }
  template <class DestT>
  inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }

private:

  /// RPC Map Transform needs to be explicitly copied and told to cache for performance.
  template <class T>
  prerasterize_type PreRasterHelper( BBox2i const& bbox, vector<T> const& transforms) const {

    // Code for NON-MAP-PROJECTED session types.
    if (m_is_map_projected == false) {
      // We explicitly bring in-memory the disparities for the current box
      // to speed up processing later, and then we pretend this is the entire
      // image by virtually enlarging it using a CropView.
      vector< ImageViewRef<DPixelT> > disparity_cropviews;
      for (int p = 0; p < (int)m_disparity_maps.size(); p++){
        ImageView<DPixelT> clip( crop( m_disparity_maps[p], bbox ) );
        ImageViewRef<DPixelT> cropview_clip = crop(clip, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
        disparity_cropviews.push_back(cropview_clip);
      }

      return prerasterize_type(disparity_cropviews, transforms, m_stereo_model, m_is_map_projected);
    }

    // Code for MAP-PROJECTED session types.

    // This is to help any transforms (right now just Map2CamTrans)
    // that must cache their side data. Normally this would happen if
    // we were using a TransformView. Copies are made of the
    // transforms so we are not having a race condition with setting
    // the cache in both transforms while the other threads want to do the same.
    // - Without some sort of duplication function in the transform base class we need
    //   to manually copy the Map2CamTrans type which is pretty hacky.
    vector<T> transforms_copy(transforms.size());
    for (size_t i = 0; i < transforms.size(); ++i) {
      transforms_copy[i] = make_transform_copy(transforms[i]);
    }
    // As a side effect this call makes transforms_copy create a local cache we want later
    transforms_copy[0]->reverse_bbox(bbox); 

    if (transforms_copy.size() != m_disparity_maps.size() + 1){
      vw_throw( ArgumentErr() << "In multi-view triangulation, "
                << "the number of disparities must be one less "
                << "than the number of images." );
    }

    vector< ImageViewRef<DPixelT> > disparity_cropviews;
    for (int p = 0; p < (int)m_disparity_maps.size(); p++){

      // We explicitly bring in-memory the disparities for the current
      // box to speed up processing later, and then we pretend this is
      // the entire image by virtually enlarging it using a CropView.
      ImageView<DPixelT> clip( crop( m_disparity_maps[p], bbox ) );
      ImageViewRef<DPixelT> cropview_clip = crop(clip, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
      disparity_cropviews.push_back(cropview_clip);

      // Work out what spots in the right image we'll be touching.
      BBox2i disparity_range = stereo::get_disparity_range(clip);
      disparity_range.max() += Vector2i(1,1);
      BBox2i right_bbox = bbox + disparity_range.min();
      right_bbox.max() += disparity_range.size();

      // Also cache the data for subsequent transforms
      // As a side effect this call makes transforms_copy create a local cache we want later
      transforms_copy[p+1]->reverse_bbox(right_bbox); 
    }

    return prerasterize_type(disparity_cropviews, transforms_copy, m_stereo_model, m_is_map_projected);
  } // End function PreRasterHelper() DGMapRPC version

}; // End class StereoTXAndErrorView

/// Just a wrapper function for StereoTXAndErrorView view construction
template <class DisparityT, class StereoModelT>
StereoTXAndErrorView<DisparityT, StereoModelT>
stereo_error_triangulate( vector<DisparityT> const& disparities,
                          vector<TXT>        const& transforms,
                          StereoModelT       const& model,
                          bool is_map_projected ) {

  typedef StereoTXAndErrorView<DisparityT, StereoModelT> result_type;
  return result_type( disparities, transforms, model, is_map_projected );
}


/// Compute an unwarped disparity image from the input disparity image
/// and the image transforms.
/// - Note that the output image size is not the same as the input disparity image.
template <typename DisparityT>
class UnalignDisparityView: public ImageViewBase<UnalignDisparityView<DisparityT> >{
  
  DisparityT const& m_disparity;
  TXT        const& m_left_transform;
  TXT        const& m_right_transform;

  ASPGlobalOptions const& m_opt;
  int m_num_cols, m_num_rows;
  bool m_is_map_projected;
  std::map <std::pair<int, int>, Vector2> m_unaligned_trans;
public:
  UnalignDisparityView( DisparityT const& disparity,
                       TXT        const& left_transform,
                       TXT        const& right_transform,
                       ASPGlobalOptions const& opt):
    m_disparity(disparity), m_left_transform(left_transform), 
    m_right_transform(right_transform), m_opt(opt),
    m_num_cols(0), m_num_rows(0), m_is_map_projected(m_opt.session->isMapProjected()){

    // Compute the output image size
    
    if (!m_is_map_projected) {
      // The left image passed as input is the original
      // unprojected/unaligned one, hence use its size.
      std::string left_file  = m_opt.in_file1;
      DiskImageView<float> left_img(left_file);
      m_num_cols = left_img.cols();
      m_num_rows = left_img.rows();
    }else{
      // Map projected, need to check all the pixel coordinates.
      // This is going to be slow for large images!
      BBox2i img_box;

      // Use sampling as this operation is very slow.
      int sample_len = 10;
      int num_min_samples = 100;
      int col_sample = std::max(1, std::min(sample_len, m_disparity.cols()/num_min_samples));
      int row_sample = std::max(1, std::min(sample_len, m_disparity.rows()/num_min_samples));

      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = col_sample / std::max(double(m_disparity.cols()), 1.0);
      tpc.report_progress(0);
      vw_out() << "\nEstimating the unaligned disparity dimensions.\n";

      for (int col = 0; col < m_disparity.cols(); col++) {

	// Ensure that the last column is picked
	if (col % col_sample != 0 && col != m_disparity.cols() - 1) 
	  continue;

        for (int row = 0; row < m_disparity.rows(); row++) {
	  
	  // Ensure that the last row is picked
	  if (row % row_sample != 0 && row != m_disparity.rows() - 1) 
	    continue;

	  // This is quite important to avoid an incorrectly computed img_box.
          typename DisparityT::pixel_type dpix = m_disparity(col, row);
          if (!is_valid(dpix))
	    continue;

          // Unalign the left pixel
	  Vector2 left_pix;
	  try{
	    left_pix  = m_left_transform->reverse(Vector2(col, row));
	  }catch(...){
	    continue;
	  }
          img_box.grow(left_pix);

	  // Save this lookup map for the future
	  m_unaligned_trans[std::make_pair(col, row)] = left_pix;
        }

	tpc.report_incremental_progress(inc_amount);
      }
      tpc.report_finished();

      // Grow the box to account for the fact that we did a sub-sampling
      // and may have missed some points.
      Vector2 diff = img_box.max() - img_box.min();
      if (!img_box.empty()) {
	img_box.grow( img_box.min() - 0.1*diff);
	img_box.grow( img_box.max() + 0.1*diff);
      }
      
      m_num_cols = img_box.max().x();
      m_num_rows = img_box.max().y();

      vw_out() << "Dimensions are: " << m_num_cols << ' ' << m_num_rows << ".\n";
    }
    // Done computing the input image size.
  }

  // ImageView interface
  typedef PixelMask<Vector2f>                          pixel_type;
  typedef pixel_type                                   result_type;
  typedef ProceduralPixelAccessor<UnalignDisparityView> pixel_accessor;

  inline int32 cols  () const { return m_num_cols; }
  inline int32 rows  () const { return m_num_rows; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "UnalignDisparityView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    TXT local_left_transform;
    TXT local_right_transform;

    // For map-projected images the transforms are not thread-safe,
    // hence need to make a copy of them.
    if (!m_is_map_projected) {
      local_left_transform = m_left_transform;
      local_right_transform = m_right_transform;
    }else{
      local_left_transform = make_transform_copy(m_left_transform);
      local_right_transform = make_transform_copy(m_right_transform);
    }

    
    // We will do some averaging
    int KERNEL_SIZE = 1;
    
    BBox2i curr_bbox = bbox;
    curr_bbox.expand(2*KERNEL_SIZE);
    curr_bbox.crop(BBox2i(0, 0, cols(), rows()));
    
    // Initialize the unaligned disparity values for this tile.
    ImageView<pixel_type> unaligned_disp(curr_bbox.width(), curr_bbox.height());
    ImageView<int> count(curr_bbox.width(), curr_bbox.height());
    for (int col = 0; col < curr_bbox.width(); col++) {
      for (int row = 0; row < curr_bbox.height(); row++) {
        unaligned_disp(col, row) = pixel_type();
        unaligned_disp(col, row).invalidate();
	count(col, row) = 0;
      }
    }
    
    // Find the bounding box of pixels we will need from the disparity image.
    // For mapprojected images the forward() function is not always accurate,
    // and it is also very slow, hence avoid it.
    BBox2i disp_bbox;
    if (!m_is_map_projected) {
      BBox2i full_disp_bbox = bounding_box(m_disparity);
      for (int col = 0; col < unaligned_disp.cols(); col++) {
	for (int row = 0; row < unaligned_disp.rows(); row++) {
	  
	  // Get the pixel coordinate in the output image (left unaligned pixel),
	  // Then get the pixel coordinate in the left input image.
	  Vector2 output_pixel(col + curr_bbox.min()[0], row + curr_bbox.min()[1]);
	  Vector2 left_aligned_pixel;
	try {
	  left_aligned_pixel = local_left_transform->forward(output_pixel);
	}catch(...){
	  // This can fail since we may apply it to pixels outside of range
	  continue;
	}
	if (!full_disp_bbox.contains(left_aligned_pixel)) 
	  continue;
	disp_bbox.grow(left_aligned_pixel);
	}
      }
    }else{
      for (int col = 0; col < m_disparity.cols(); col++) {
	for (int row = 0; row < m_disparity.rows(); row++) {
	  
	  std::pair<int, int> pix = std::make_pair(col, row);
	  std::map <std::pair<int, int>, Vector2>::const_iterator it = m_unaligned_trans.find(pix);
	  if (it == m_unaligned_trans.end())
	    continue;

	  Vector2 rev = it->second;
	  if (curr_bbox.contains(rev)) {
	    disp_bbox.grow(Vector2(col, row));
	  }
	}
      }

      // Grow the box to account for the fact that we did a sub-sampling
      // and may have missed some points.
      Vector2 diff = disp_bbox.max() - disp_bbox.min();
      if (!disp_bbox.empty()) {
	disp_bbox.grow( disp_bbox.min() - 0.1*diff);
	disp_bbox.grow( disp_bbox.max() + 0.1*diff);
      }
      
    }
    
    // Expand to take into account the sampling to be used below
    disp_bbox.expand(2*KERNEL_SIZE);

    // Crop to its maximum extent
    disp_bbox.crop(bounding_box(m_disparity));

    // Rasterize the section of the disparity image that we need for this tile
    typedef typename DisparityT::pixel_type DispPixelT;
    ImageView<DispPixelT> disp = crop(m_disparity, disp_bbox);

    for (int col = 0; col < disp.cols(); col++) {
      for (int row = 0; row < disp.rows(); row++) {
	
	DispPixelT dpix = disp(col, row);
	if (!is_valid(dpix))
	  continue;

	// Go from position in the cropped disparity to the
	// position in the full disparity.
	int ucol = col + disp_bbox.min().x();
	int urow = row + disp_bbox.min().y();
	
	// De-warp left and right pixels to be in the camera coordinate system
	Vector2 left_pix, right_pix;
	try{
	  left_pix  = local_left_transform->reverse ( Vector2(ucol, urow) );
	  right_pix = local_right_transform->reverse( Vector2(ucol, urow)
							  + stereo::DispHelper(dpix) );
	}catch(...){
	  continue;
	}
	Vector2 dir = right_pix - left_pix; // disparity value
	
	// This averaging is useful in filling tiny holes and avoiding staircasing.
	// TODO: Use some weights. The closer contribution should have more weight.
	for (int icol = -KERNEL_SIZE; icol <= KERNEL_SIZE; icol++) {
	  for (int irow = -KERNEL_SIZE; irow <= KERNEL_SIZE; irow++) {
	    int lcol = round(left_pix[0]) + icol;
	    int lrow = round(left_pix[1]) + irow;
	    
	    // shift to be in the domain of the cropped image
	    lcol -= curr_bbox.min()[0];
	    lrow -= curr_bbox.min()[1];
	    if (lcol < 0 || lcol >= curr_bbox.width())  continue;
	    if (lrow < 0 || lrow >= curr_bbox.height()) continue;
	    if (!is_valid(unaligned_disp(lcol, lrow)))
	      unaligned_disp(lcol, lrow).validate();
	    unaligned_disp(lcol, lrow).child() += dir;
	    count(lcol, lrow)++;
	  }
	}
	
      }
    }
    
    for (int col = 0; col < unaligned_disp.cols(); col++) {
      for (int row = 0; row < unaligned_disp.rows(); row++) {
	if (count(col, row) == 0)
	  unaligned_disp(col, row).invalidate();
	else
	  unaligned_disp(col, row) /= double(count(col, row));
      }
    }
    
    // Use the crop trick to fake that the support region is the same size as the entire image.
    return prerasterize_type(unaligned_disp, -curr_bbox.min().x(), -curr_bbox.min().y(),
			     cols(), rows());
  }
  
  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
}; // End class UnalignDisparityView

// Take a given disparity and make it between the original unaligned images
template <class DisparityT>
void unalign_disparity(vector<ASPGlobalOptions> const& opt_vec,
                       vector<DisparityT> const& disparities,
                       vector<TXT>        const& transforms,
                       std::string        const& disp_file) {
  
  // This function is fairly specialized!
  VW_ASSERT( disparities.size() == 1 && transforms.size() == 2,
               vw::ArgumentErr() << "Expecting two images and one disparity.\n" );
  DisparityT const& disp = disparities[0]; // pull the disparity

  // Transforms to compensate for alignment
  TXT left_trans  = transforms[0];
  TXT right_trans = transforms[1];

  // Special case to overwrite the transforms
  ASPGlobalOptions opt = opt_vec[0];
  bool usePinholeEpipolar = ( (stereo_settings().alignment_method == "epipolar") &&
                              ( opt.session->name() == "pinhole" ||
                                opt.session->name() == "nadirpinhole") );
  if (usePinholeEpipolar) {
    StereoSessionPinhole* pinPtr = dynamic_cast<StereoSessionPinhole*>(opt.session.get());
    if (pinPtr == NULL) 
      vw_throw(ArgumentErr() << "Expected a pinhole camera.\n");
    pinPtr->pinhole_cam_trans(left_trans, right_trans);
  }

  bool is_map_projected = opt.session->isMapProjected();
  Stopwatch sw;
  sw.start();

  cartography::GeoReference left_georef;
  bool   has_left_georef = false;
  bool   has_nodata      = false;
  double nodata          = -32768.0;
  vw_out() << "Unaligning the disparity.\n";
  vw_out() << "Writing: " << disp_file << "\n";
  vw::cartography::block_write_gdal_image(disp_file, 
					  UnalignDisparityView<DisparityT>(disparities[0],
									   left_trans, 
									   right_trans, opt),
					  has_left_georef, left_georef,
					  has_nodata, nodata, opt,
					  TerminalProgressCallback("asp", "\t--> Undist disp:") );
  
  sw.stop();
  vw_out() << "Unaligning disparity elapsed time: " << sw.elapsed_seconds() << " seconds.\n";
  
}

/// Bin the disparities, and from each bin get a disparity value.
/// This will create a correspondence from the left to right image,
/// which we save in the match format.
/// When gen_triplets is true, and there are many overlapping images,
/// try hard to have many IP with the property that each such IP is seen
/// in more than two images. This helps with bundle adjustment.
template <class DisparityT>
void compute_matches_from_disp(vector<ASPGlobalOptions> const& opt_vec,
                               vector<DisparityT> const& disparities,
                               vector<TXT>        const& transforms,
                               std::string        const& match_file,
                               int                const  max_num_matches,
                               bool                      gen_triplets
                               ) {

  VW_ASSERT( disparities.size() == 1 && transforms.size() == 2,
               vw::ArgumentErr() << "Expecting two images and one disparity.\n" );
  DisparityT const& disp = disparities[0]; // pull the disparity

  // Transforms to compensate for alignment
  TXT left_trans  = transforms[0];
  TXT right_trans = transforms[1];

  bool usePinholeEpipolar = ( (stereo_settings().alignment_method == "epipolar") &&
                              ( opt_vec[0].session->name() == "pinhole" ||
                                opt_vec[0].session->name() == "nadirpinhole") );
  if (usePinholeEpipolar) {
    StereoSessionPinhole* pinPtr = dynamic_cast<StereoSessionPinhole*>(opt_vec[0].session.get());
    if (pinPtr == NULL) 
      vw_throw(ArgumentErr() << "Expected a pinhole camera.\n");
    pinPtr->pinhole_cam_trans(left_trans, right_trans);
  }

  std::vector<vw::ip::InterestPoint> left_ip, right_ip;

  if (!gen_triplets) {

    double num_pixels = double(disp.cols()) * double(disp.rows());
    double bin_len = sqrt(num_pixels/std::min(double(max_num_matches), num_pixels));
    VW_ASSERT( bin_len >= 1.0, vw::ArgumentErr() << "Expecting bin_len >= 1.\n" );

    int lenx = round( disp.cols()/bin_len ); lenx = std::max(1, lenx);
    int leny = round( disp.rows()/bin_len ); leny = std::max(1, leny);

    // Iterate over bins.

    vw_out() << "Computing interest point matches based on disparity.\n";
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    double inc_amount = 1.0 / double(lenx);
    tpc.report_progress(0);

    for (int binx = 0; binx < lenx; binx++) {

      // Pick the disparity at the center of the bin
      int posx = round( (binx+0.5)*bin_len );

      for (int biny = 0; biny < leny; biny++) {

        int posy = round( (biny+0.5)*bin_len );

        if (posx >= disp.cols() || posy >= disp.rows()) 
          continue;
        typedef typename DisparityT::pixel_type DispPixelT;
        DispPixelT dpix = disp(posx, posy);
        if (!is_valid(dpix))
          continue;

        // De-warp left and right pixels to be in the camera coordinate system
        Vector2 left_pix  = left_trans->reverse ( Vector2(posx, posy) );
        Vector2 right_pix = right_trans->reverse( Vector2(posx, posy) + stereo::DispHelper(dpix) );

        left_ip.push_back(ip::InterestPoint(left_pix.x(), left_pix.y()));
        right_ip.push_back(ip::InterestPoint(right_pix.x(), right_pix.y()));
      }

      tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();

  } else{

    // First create ip with left_ip being at integer multiple of bin size.
    // Then do the same for right_ip. This way there is a symmetry
    // and predictable location for ip. So if three images overlap,
    // a feature can often be seen in many of them whether a given
    // image is left in some pairs or right in some others.

    // Note that the code above is modified in subtle ways.

    // Need these to not insert an ip twice, as then bundle_adjust
    // will wipe both copies
    std::map<double, double> left_done, right_done;
    
    // Start with the left
    {
      DiskImageView<float> left_img(opt_vec[0].in_file1);
    
      double num_pixels = double(left_img.cols()) * double(left_img.rows());
      int bin_len = round(sqrt(num_pixels/std::min(double(max_num_matches), num_pixels)));
      VW_ASSERT( bin_len >= 1, vw::ArgumentErr() << "Expecting bin_len >= 1.\n" );

      int lenx = round( left_img.cols()/bin_len ); lenx = std::max(1, lenx);
      int leny = round( left_img.rows()/bin_len ); leny = std::max(1, leny);

      // Iterate over bins.

      vw_out() << "Computing interest point matches based on disparity.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = 1.0 / double(lenx);
      tpc.report_progress(0);

      for (int binx = 0; binx <= lenx; binx++) {

        int posx = binx*bin_len; // integer multiple of bin length

        for (int biny = 0; biny <= leny; biny++) {

          int posy = biny*bin_len; // integer multiple of bin length

          if (posx >= left_img.cols() || posy >= left_img.rows()) 
            continue;

          Vector2 left_pix(posx, posy);
          Vector2 trans_left_pix, trans_right_pix, right_pix;
        
          typedef typename DisparityT::pixel_type DispPixelT;

          // Make the left pixel go to the disparity domain. Find the corresponding
          // right pixel. And make that one go to the right image domain.
          trans_left_pix = round(left_trans->forward(left_pix));
          if (trans_left_pix[0] < 0 || trans_left_pix[0] >= disp.cols()) continue;
          if (trans_left_pix[1] < 0 || trans_left_pix[1] >= disp.rows()) continue;
          DispPixelT dpix = disp(trans_left_pix[0], trans_left_pix[1]);
          if (!is_valid(dpix))
            continue;
          trans_right_pix = trans_left_pix + stereo::DispHelper(dpix);
          right_pix = right_trans->reverse(trans_right_pix);

          // Add this ip unless found already. This is clumsy, but we
          // can't use a set since there is no ordering for pairs.
          std::map<double, double>::iterator it;
          it = left_done.find(left_pix.x());
          if (it != left_done.end() && it->second == left_pix.y()) continue; 
          it = right_done.find(right_pix.x());
          if (it != right_done.end() && it->second == right_pix.y()) continue; 
          left_done[left_pix.x()] = left_pix.y();
          right_done[right_pix.x()] = right_pix.y();
          ip::InterestPoint lip(left_pix.x(), left_pix.y());
          ip::InterestPoint rip(right_pix.x(), right_pix.y());
          left_ip.push_back(lip); 
          right_ip.push_back(rip);
        }
        
        tpc.report_incremental_progress( inc_amount );
      }
      tpc.report_finished();
    }
    
    // Now create ip in predictable location for the right image.This is hard,
    // as the disparity goes from left to right, so we need to examine every disparity.
    typedef typename DisparityT::pixel_type DispPixelT;
    ImageView<DispPixelT> disp_copy = copy(disp);
    {
      DiskImageView<float> right_img(opt_vec[0].in_file2);
    
      double num_pixels = double(right_img.cols()) * double(right_img.rows());
      int bin_len = round(sqrt(num_pixels/std::min(double(max_num_matches), num_pixels)));
      VW_ASSERT( bin_len >= 1, vw::ArgumentErr() << "Expecting bin_len >= 1.\n" );

      // Iterate over disparity.

      vw_out() << "Doing a second pass. This will be very slow.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = 1.0 / double(disp_copy.cols());
      tpc.report_progress(0);

      for (int col = 0; col < disp_copy.cols(); col++) {
        for (int row = 0; row < disp_copy.rows(); row++) {

          Vector2 trans_left_pix(col, row);
          Vector2 left_pix, trans_right_pix, right_pix;

          DispPixelT dpix = disp_copy(trans_left_pix[0], trans_left_pix[1]);
          if (!is_valid(dpix))
            continue;

          // Compute the left and right pixels. 
          left_pix        = left_trans->reverse(trans_left_pix);
          trans_right_pix = trans_left_pix + stereo::DispHelper(dpix);
          right_pix       = right_trans->reverse(trans_right_pix);


          // If the right pixel is a multiple of the bin size, keep
          // it.
          right_pix = round(right_pix); // very important
          if ( int(right_pix[0]) % bin_len != 0 ) continue;
          if ( int(right_pix[1]) % bin_len != 0 ) continue;

          // Add this ip unless found already. This is clumsy, but we
          // can't use a set since there is no ordering for pairs.
          std::map<double, double>::iterator it;
          it = left_done.find(left_pix.x());
          if (it != left_done.end() && it->second == left_pix.y()) continue; 
          it = right_done.find(right_pix.x());
          if (it != right_done.end() && it->second == right_pix.y()) continue; 
          left_done[left_pix.x()] = left_pix.y();
          right_done[right_pix.x()] = right_pix.y();
          ip::InterestPoint lip(left_pix.x(), left_pix.y());
          ip::InterestPoint rip(right_pix.x(), right_pix.y());
          left_ip.push_back(lip); 
          right_ip.push_back(rip);
        }
        
        tpc.report_incremental_progress( inc_amount );
      }
      tpc.report_finished();
    }

    
  } // end considering multi-image friendly ip

  vw_out() << "Determined " << left_ip.size()
           << " interest point matches from disparity.\n";

  vw_out() << "Writing: " << match_file << std::endl;
  ip::write_binary_match_file(match_file, left_ip, right_ip);
}

namespace asp{

  // TODO: Move some of these functions to a class or something!

  // ImageView operator that takes the last three elements of a vector
  // (the error part) and replaces them with the norm of that 3-vector.
  struct PointAndErrorNorm : public ReturnFixedType<Vector4> {
    Vector4 operator() (Vector6 const& pt) const {
      Vector4 result;
      subvector(result,0,3) = subvector(pt,0,3);
      result[3] = norm_2(subvector(pt,3,3));
      return result;
    }
  };
  template <class ImageT>
  UnaryPerPixelView<ImageT, PointAndErrorNorm>
  inline point_and_error_norm( ImageViewBase<ImageT> const& image ) {
    return UnaryPerPixelView<ImageT, PointAndErrorNorm>( image.impl(),
                                                         PointAndErrorNorm() );
  }

  template <class ImageT>
  void save_point_cloud(Vector3 const& shift, ImageT const& point_cloud,
                        string const& point_cloud_file,
                        ASPGlobalOptions const& opt){

    vw_out() << "Writing point cloud: " << point_cloud_file << "\n";
    bool has_georef = true;
    cartography::GeoReference georef = opt.session->get_georef();

    bool has_nodata = false;
    double nodata = -std::numeric_limits<float>::max(); // smallest float

    if ( opt.session->supports_multi_threading() ){
      asp::block_write_approx_gdal_image
        ( point_cloud_file, shift,
          stereo_settings().point_cloud_rounding_error,
          point_cloud,
          has_georef, georef, has_nodata, nodata,
          opt, TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }else{
      // ISIS does not support multi-threading
      asp::write_approx_gdal_image
        ( point_cloud_file, shift,
          stereo_settings().point_cloud_rounding_error,
          point_cloud,
          has_georef, georef, has_nodata, nodata,
          opt, TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }

  }

  Vector3 find_approx_points_median(vector<Vector3> const& points){

    // Find the median of the x coordinates of points, then of y, then of
    // z. Perturb the median a bit to ensure it is never exactly on top
    // of a real point, as in such a case after subtraction of that
    // point from median we'd get the zero vector which by convention
    // is invalid.

    if (points.empty())
      return Vector3();

    Vector3 median;
    vector<double> V(points.size());
    for (int i = 0; i < (int)median.size(); i++){
      for (int p = 0; p < (int)points.size(); p++) V[p] = points[p][i];
      sort(V.begin(), V.end());
      median[i] = V[points.size()/2];

      median[i] += median[i]*1e-10*rand()/double(RAND_MAX);
    }

    return median;
  }

  Vector3 find_point_cloud_center(Vector2i const& tile_size,
                                  ImageViewRef<Vector6> const& point_cloud){

    // Compute the point cloud in a tile around the center of the
    // cloud. Find the median of all the points in that cloud.  That
    // will be the cloud center. If the tile is too small, spiral away
    // from the center adding other tiles.  Keep the tiles aligned to a
    // multiple of tile_size, for consistency with how the point cloud
    // is written to disk later on.

    int numx = (int)ceil(point_cloud.cols()/double(tile_size[0]));
    int numy = (int)ceil(point_cloud.rows()/double(tile_size[1]));

    vector<Vector3> points;
    for (int r = 0; r <= max(numx/2, numy/2); r++){
      // We are now on the boundary of the square of size 2*r with
      // center at (numx/2, numy/2). Iterate over that boundary.
      for (int x = numx/2-r; x <= numx/2+r; x++){
        for (int y = numy/2-r; y <= numy/2+r; y++){

          if ( x != numx/2-r && x != numx/2+r &&
               y != numy/2-r && y != numy/2+r)
            continue; // skip inner points

          if (x < 0 || y < 0 || x >= numx || y >= numy)
            continue; // out of bounds

          BBox2i box(x*tile_size[0], y*tile_size[1], tile_size[0], tile_size[1]);
          box.crop(bounding_box(point_cloud));

          // Crop to the cloud area actually having points
          box.crop(stereo_settings().trans_crop_win);

          // Triangulate in the existing box
          ImageView<Vector6> cropped_cloud = crop(point_cloud, box);
          for (int px = 0; px < cropped_cloud.cols(); px++){
            for (int py = 0; py < cropped_cloud.rows(); py++){
              Vector3 xyz = subvector(cropped_cloud(px, py), 0, 3);
              if (xyz == Vector3())
                continue;
              points.push_back(xyz);
            }
          }

          // Stop if we have enough points to do a reliable mean estimation
          if (points.size() > 100)
            return find_approx_points_median(points);

        }// end y loop
      }// end x loop
    }// end r loop

    // Have to use what we've got
    return find_approx_points_median(points);
  }

  bool read_point(string const& file, Vector3 & point){

    point = Vector3();

    ifstream fh(file.c_str());
    if (!fh.good()) return false;

    for (int c = 0; c < (int)point.size(); c++)
      if (! (fh >> point[c]) ) return false;

    return true;
  }

  void write_point(string const& file, Vector3 const& point){

    ofstream fh(file.c_str());
    fh.precision(18); // precision(16) is not enough
    for (int c = 0; c < (int)point.size(); c++)
      fh << point[c] << " ";
    fh << endl;

  }

} // End namespace asp

/// Main triangulation function
void stereo_triangulation( string          const& output_prefix,
                           vector<ASPGlobalOptions> const& opt_vec ) {

  typedef          StereoSession                       SessionT;
  typedef          ImageViewRef<PixelMask<Vector2f> >  PVImageT;
  typedef typename SessionT::stereo_model_type         StereoModelT;

  try { // Outer try/catch

    const bool is_map_projected = opt_vec[0].session->isMapProjected();

    // Collect the images, cameras, and transforms. The left image is
    // the same in all n-1 stereo pairs forming the n images multiview
    // system. Same for cameras and transforms.
    vector<string> image_files, camera_files;
    vector< boost::shared_ptr<camera::CameraModel> > cameras;
    vector<typename SessionT::tx_type> transforms;
    for (int p = 0; p < (int)opt_vec.size(); p++){

      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt_vec[p].session->camera_models(camera_model1, camera_model2);

      //boost::shared_ptr<SessionT> sPtr = boost::dynamic_pointer_cast<SessionT>(opt_vec[p].session);
      boost::shared_ptr<SessionT> sPtr = opt_vec[p].session;

      if (p == 0){ // The first image is the "left" image for all pairs.
        image_files.push_back(opt_vec[p].in_file1);
        camera_files.push_back(opt_vec[p].cam_file1);
        cameras.push_back(camera_model1);
        transforms.push_back(sPtr->tx_left());
      }

      image_files.push_back(opt_vec[p].in_file2);
      camera_files.push_back(opt_vec[p].cam_file2);
      cameras.push_back(camera_model2);
      transforms.push_back(sPtr->tx_right());
    }

    // If the distance from the left camera center to a point is
    // greater than the universe radius, we remove that pixel and
    // replace it with a zero vector, which is the missing pixel value in the point_image.
    //
    // We apply the universe radius here and then write the result directly to a file on disk.
    stereo::UniverseRadiusFunc universe_radius_func(Vector3(),0,0);
    try{
      if ( stereo_settings().universe_center == "camera" ) {
        if (opt_vec[0].session->name() == "rpc") {
          vw_throw(InputErr() << "Stereo with RPC cameras cannot "
                              << "have the camera as the universe center.\n");
        }

        universe_radius_func = stereo::UniverseRadiusFunc(cameras[0]->camera_center(Vector2()),
                                                          stereo_settings().near_universe_radius,
                                                          stereo_settings().far_universe_radius);
      } else if ( stereo_settings().universe_center == "zero" ) {
        universe_radius_func = stereo::UniverseRadiusFunc(Vector3(),
                                                          stereo_settings().near_universe_radius,
                                                          stereo_settings().far_universe_radius);
      }
    } catch (std::exception &e) {
      vw_out() << e.what() << std::endl;
      vw_out(WarningMessage) << "Could not find the camera center. "
                             << "Will not be able to filter triangulated points by radius.\n";
    } // End try/catch

    vector<PVImageT> disparity_maps;
    for (int p = 0; p < (int)opt_vec.size(); p++){
      disparity_maps.push_back(opt_vec[p].session->pre_pointcloud_hook(opt_vec[p].out_prefix+"-F.tif"));
    }

    // Create a disparity map with between the original unalinged images 
    if (stereo_settings().unalign_disparity) {
      std::string unalign_disp = asp::unwarped_disp_file(output_prefix,
                                                         opt_vec[0].in_file1, 
                                                         opt_vec[0].in_file2);
      unalign_disparity(opt_vec, disparity_maps, transforms, unalign_disp);
    }
    
    std::string match_file = ip::match_filename(output_prefix + "-disp",
                                                opt_vec[0].in_file1, 
                                                opt_vec[0].in_file2);

    // Pull matches from disparity. Highly experimental.
    if (stereo_settings().num_matches_from_disparity > 0 && 
        stereo_settings().num_matches_from_disp_triplets > 0) {
      vw_throw( ArgumentErr() << "Cannot have both --num-matches-from-disparity and  "
                              << "--num-matches-from-disp-triplets.\n" );
    }

    if (stereo_settings().num_matches_from_disparity > 0) {
      bool gen_triplets = false;
      compute_matches_from_disp(opt_vec, disparity_maps, transforms, match_file,
                                stereo_settings().num_matches_from_disparity, gen_triplets);
    }
    if (stereo_settings().num_matches_from_disp_triplets > 0) {
      bool gen_triplets = true;
      compute_matches_from_disp(opt_vec, disparity_maps, transforms, match_file,
                                stereo_settings().num_matches_from_disp_triplets, gen_triplets);
    }
    
    // Piecewise adjustments for jitter
    if (stereo_settings().image_lines_per_piecewise_adjustment > 0 &&
        !stereo_settings().skip_computing_piecewise_adjustments){

      // TODO: This must be proportional to how many adjustments have!
      double max_num_matches = stereo_settings().num_matches_for_piecewise_adjustment;

      bool gen_triplets = false;
      compute_matches_from_disp(opt_vec, disparity_maps, transforms, match_file,
                                max_num_matches, gen_triplets);

      int num_threads = opt_vec[0].num_threads;
      asp::jitter_adjust(image_files, camera_files, cameras,
                         output_prefix, opt_vec[0].session->name(),
                         match_file,  num_threads);
      //asp::ccd_adjust(image_files, camera_files, cameras, output_prefix,
      //                match_file,  num_threads);
    }

    if (stereo_settings().compute_piecewise_adjustments_only) {
      vw_out() << "Computed the piecewise adjustments. Will stop here." << endl;
      return;
    }

    // Reload the cameras, loading the piecewise corrections for jitter.
    if (stereo_settings().image_lines_per_piecewise_adjustment > 0) {

      stereo_settings().bundle_adjust_prefix = output_prefix; // trigger loading adj cams
      cameras.clear();
      for (int p = 0; p < (int)opt_vec.size(); p++){

        boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
        opt_vec[p].session->camera_models(camera_model1, camera_model2);
        if (p == 0) // The first image is the "left" image for all pairs.
          cameras.push_back(camera_model1);
        cameras.push_back(camera_model2);
      }
    }

    if (is_map_projected)
      vw_out() << "\t--> Inputs are map projected" << std::endl;

    // Strip the smart pointers and form the stereo model
    std::vector<const vw::camera::CameraModel *> camera_ptrs;
    int num_cams = cameras.size();
    for (int c = 0; c < num_cams; c++) {
      camera_ptrs.push_back(cameras[c].get());
    }

    // Convert the angle tol to be in terms of dot product and pass it
    // to the stereo model.
    double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
    StereoModelT stereo_model( camera_ptrs, stereo_settings().use_least_squares,
                               angle_tol);

    // Apply radius function and stereo model in one go
    vw_out() << "\t--> Generating a 3D point cloud." << endl;
    ImageViewRef<Vector6> point_cloud = per_pixel_filter
      (stereo_error_triangulate
       (disparity_maps, transforms, stereo_model, is_map_projected),
       universe_radius_func);

    // If we crop the left and right images, at each run we must
    // recompute the cloud center, as the cropping windows may have changed.
    bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

    // Compute the point cloud center, unless done by now
    Vector3 cloud_center = Vector3();
    if (!stereo_settings().save_double_precision_point_cloud){
      string cloud_center_file = output_prefix + "-PC-center.txt";
      if (!read_point(cloud_center_file, cloud_center) || crop_left || crop_right){
        if (!stereo_settings().skip_point_cloud_center_comp) {
          cloud_center = find_point_cloud_center(opt_vec[0].raster_tile_size, point_cloud);
          write_point(cloud_center_file, cloud_center);
        }
      }
    }
    if (stereo_settings().compute_point_cloud_center_only){
      vw_out() << "Computed the point cloud center. Will stop here." << endl;
      return;
    }

    // We are supposed to do the triangulation in trans_crop_win only
    // so force rasterization in that box only using crop().
    BBox2i cbox = stereo_settings().trans_crop_win;
    string point_cloud_file = output_prefix + "-PC.tif";
    if (stereo_settings().compute_error_vector){

      if (num_cams > 2)
        vw_out(WarningMessage) << "For more than two cameras, the error "
                               << "vector between rays is not meaningful. "
                               << "Setting it to (err_len, 0, 0)." << endl;

      ImageViewRef<Vector6> crop_pc = crop(point_cloud, cbox);
      save_point_cloud(cloud_center, crop_pc, point_cloud_file, opt_vec[0]);
    }else{
      ImageViewRef<Vector4> crop_pc = crop(point_and_error_norm(point_cloud), cbox);
      save_point_cloud(cloud_center, crop_pc, point_cloud_file, opt_vec[0]);
    } // End if/else

    // Must print this at the end, as it contains statistics on the number of rejected points.
    vw_out() << "\t--> " << universe_radius_func;

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage "
              << "-- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  } // End outer try/catch
} // End function stereo_triangulation()


int main( int argc, char* argv[] ) {

  try {
    xercesc::XMLPlatformUtils::Initialize();

    vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 4 --> TRIANGULATION \n";

    stereo_register_sessions();

    // Unlike other stereo executables, triangulation can handle multiple images and cameras.
    bool verbose = false;
    vector<ASPGlobalOptions> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, TriangulationDescription(),
                         verbose, output_prefix, opt_vec);

    if (opt_vec.size() > 1){
      // For multiview, turn on logging to file in the run directory
      // in output_prefix, not just in individual subdirectories.
      asp::log_to_file(argc, argv, opt_vec[0].stereo_default_filename,
                       output_prefix);
    }

    // Keep only those stereo pairs for which filtered disparity exists
    vector<ASPGlobalOptions> opt_vec_new;
    for (int p = 0; p < (int)opt_vec.size(); p++){
      if (fs::exists(opt_vec[p].out_prefix+"-F.tif"))
        opt_vec_new.push_back(opt_vec[p]);
    }
    opt_vec = opt_vec_new;
    if (opt_vec.empty())
      vw_throw( ArgumentErr() << "No valid F.tif files found.\n" );

    // Triangulation uses small tiles.
    //---------------------------------------------------------
    int ts = ASPGlobalOptions::tri_tile_size();
    for (int s = 0; s < (int)opt_vec.size(); s++)
      opt_vec[s].raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------

    stereo_triangulation(output_prefix, opt_vec);

    vw_out() << "\n[ " << current_posix_time_string() << " ] : TRIANGULATION FINISHED \n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}
