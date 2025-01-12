// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  https://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

// TODO(oalexan1): Reconcile this with MainWidget.cc so that different
// images and curves can be overlayed while using a colormap and axes.

/// \file ColorAxes.cc

#include <qwt_color_map.h>
#include <qwt_plot_spectrogram.h>
#include <qwt_scale_widget.h>
#include <qwt_scale_draw.h>
#include <qwt_plot_zoomer.h>
#include <qwt_plot_panner.h>
#include <qwt_plot_layout.h>
#include <qwt_plot_renderer.h>
#include <qwt_plot_canvas.h>

#include <asp/GUI/ColorAxes.h>
#include <asp/GUI/GuiUtilities.h>
#include <asp/GUI/GuiBase.h>
#include <asp/Core/StereoSettings.h>
#include <vw/Image/Colormap.h> // colormaps supported by ASP

namespace vw { namespace gui {

// Small auxiliary function
inline QColor rgb2color(vw::cm::Vector3u const& c) {
  return QColor(c[0], c[1], c[2]);
}

// Colormap based on a lookup table (lut)  
class LutColormap: public QwtLinearColorMap {
public:

  // Default constructor; will not be used
  LutColormap() {} 

  // Custom constructor
  LutColormap(std::map<float, vw::cm::Vector3u> const& lut_map) {

    // Sanity check: the first and last color keys must be 0 and 1.
    if (lut_map.empty() || lut_map.begin()->first != 0.0 || lut_map.rbegin()->first != 1.0)
      popUp("First colormap stop must be at 0.0 and last at 1.0.");

    // Must replace the automatically initialized endpoints
    setColorInterval(rgb2color(lut_map.begin()->second), rgb2color(lut_map.rbegin()->second));
    
    for (auto it = lut_map.begin(); it != lut_map.end(); it++) {
      
      if (it->first == 0.0 || it->first == 1.0) 
        continue; // endpoints already added
      
      auto const& c = it->second; // c has 3 indices between 0 and 255
      addColorStop(it->first, rgb2color(it->second));
    }
  }
  
};
  
class ColorAxesZoomer: public QwtPlotZoomer {
public:
  ColorAxesZoomer(QWidget *canvas): QwtPlotZoomer(dynamic_cast<QwtPlotCanvas *>(canvas)) {
    setTrackerMode(AlwaysOff);
  }
  
};

// Manages the image to display for the ColorAxes widget 
class ColorAxesData: public QwtRasterData {

private:
  
  void calcLowResMinMax(double& min_val, double& max_val) const {
    
    // TODO(oalexan1): How about removing a small percentile of intensity from ends?

    // Get the lowest-resolution image version from the pyramid
    ImageView<double> lowres_img = m_image.img.m_img_ch1_double.pyramid().back();

    min_val = std::numeric_limits<double>::max();
    max_val = -min_val;
    for (int col = 0; col < lowres_img.cols(); col++) {
      for (int row = 0; row < lowres_img.rows(); row++) {
        
        double val = lowres_img(col, row);
        if (val == m_nodata_val) 
          continue;
        
        min_val = std::min(min_val, val);
        max_val = std::max(max_val, val);
      }
    }

    if (min_val > max_val) {
      // If the image turned out to be empty
      min_val = m_nodata_val;
      max_val = m_nodata_val;
    }
    
    return;
  }
  
public:

  ColorAxesData(imageData & image): m_image(image) {
    // TODO(oalexan1): Need to handle georeferences.

    vw::mosaic::DiskImagePyramid<double> & img = image.img.m_img_ch1_double;
    
    if (img.planes() != 1) {
      // This will be caught before we get here, but is good to have for extra
      // robustness.
      popUp("Only images with one channel can be colorized.");
      return;
    }

    // Some initializations
    m_nodata_val = img.get_nodata_val();
    m_sub_scale = -1;  // This must be set properly before being used
    m_beg_x = 0;
    m_beg_y = 0;
    
    // TODO(oalexan1): Handle no-data properly by using transparent pixels.
    m_min_val = asp::stereo_settings().min;
    m_max_val = asp::stereo_settings().max;
    if (std::isnan(m_min_val) || std::isnan(m_max_val)) // if the user did not set these
      calcLowResMinMax(m_min_val, m_max_val);

    setInterval(Qt::XAxis, QwtInterval(0, img.cols() - 1));
    setInterval(Qt::YAxis, QwtInterval(0, img.rows() - 1));
    setInterval(Qt::ZAxis, QwtInterval(m_min_val, m_max_val));
  }

  // Given that we plan to render a portion of an image on disk within these
  // bounds, and resulting in a given image size, read from disk a clip with resolution
  // and extent just enough for the job, or a little higher res and bigger.
  void prepareClip(double x0, double y0, double x1, double y1, QSize const& imageSize) {

    // Note that y0 and y1 are normally flipped
    int beg_x = floor(std::min(x0, x1)), end_x = ceil(std::max(x0, x1));
    int beg_y = floor(std::min(y0, y1)), end_y = ceil(std::max(y0, y1));

    // if in doubt, go with lower sub_scale, so higher resolution.
    double sub_scale = std::min((end_x - beg_x) / imageSize.width(),
                            (end_y - beg_y) / imageSize.height());
      
    m_level = m_image.img.m_img_ch1_double.pyramidLevel(sub_scale);
    m_sub_scale = round(pow(2.0, m_level));

    beg_x = floor(beg_x/m_sub_scale); end_x = ceil(end_x/m_sub_scale);
    beg_y = floor(beg_y/m_sub_scale); end_y = ceil(end_y/m_sub_scale);

    vw::BBox2i box;
    box.min() = Vector2i(beg_x, beg_y);
    box.max() = Vector2i(end_x + 1, end_y + 1); // because max is exclusive
    box.crop(vw::bounding_box(m_image.img.m_img_ch1_double.pyramid()[m_level]));

    // Instead of returning image(x, y), we will return
    // sub_image(x/scale + beg_x, y/scale + beg_y).
    m_sub_image = crop(m_image.img.m_img_ch1_double.pyramid()[m_level], box);

    m_beg_x = box.min().x();
    m_beg_y = box.min().y();
    
  }
  
  virtual double value(double x, double y) const {
    
    // Instead of returning m_image(x, y), we will return
    // m_sub_image(x/m_sub_scale - m_beg_x, y/m_sub_scale - m_beg_y).
    if (m_sub_scale <= 0) 
      vw::vw_throw(vw::ArgumentErr() << "Programmer error. Not ready yet to render the image.\n");
    
    vw::mosaic::DiskImagePyramid<double> const& img = m_image.img.m_img_ch1_double;

    // Return pixels at the appropriate level of resolution
    x = round(x/m_sub_scale) - m_beg_x;
    y = round(y/m_sub_scale) - m_beg_y;

    if (x < 0 || y < 0 || 
        x > m_sub_image.cols() - 1 || y > m_sub_image.rows() - 1)
      return m_min_val;
    
    double val = m_sub_image(x, y);
    
    if (val == m_nodata_val || val != val) 
      return m_min_val; // TODO(oalexan1): Make transparent

    return val;
  }
  
private:
  
  imageData & m_image;
  double m_min_val, m_max_val, m_nodata_val;

  // Can get away by using a lower-res image version at this sub scale.
  int m_level, m_beg_x, m_beg_y;
  double m_sub_scale;
  ImageView<double> m_sub_image;
};

// Manages the plotting of the data at the correct resolution level
// Sources: https://qwt.sourceforge.io/qwt__plot__spectrogram_8cpp_source.html
//          https://qwt.sourceforge.io/qwt__plot__spectrogram_8h_source.html
class ColorAxesPlotter: public QwtPlotSpectrogram {
public:
  ColorAxesPlotter(ColorAxesData * data, const QString& title = QString()):
    m_data(data), QwtPlotSpectrogram(title) {}
  
 virtual void draw(QPainter * painter,
                   const QwtScaleMap & xMap,
                   const QwtScaleMap & yMap,
                   const QRectF      & canvasRect) const {
   
   QwtPlotSpectrogram::draw(painter, xMap, yMap, canvasRect);
 }

  virtual QImage renderImage(const QwtScaleMap & xMap,
                             const QwtScaleMap & yMap,
                             const QRectF & area, 
                             const QSize & imageSize) const {

    // Based on size of the rendered image, determine the appropriate level of
    // resolution and extent to read from disk. This greatly helps with
    // reducing memory usage and latency.
    m_data->prepareClip(xMap.invTransform(0),
                        yMap.invTransform(0),
                        xMap.invTransform(imageSize.width()),
                        yMap.invTransform(imageSize.height()),
                        imageSize);

    return QwtPlotSpectrogram::renderImage(xMap, yMap, area, imageSize);
  }
  
  ColorAxesData * m_data;  
};
  
ColorAxes::ColorAxes(QWidget *parent, imageData & image):
  QwtPlot(parent), m_image(image) {

  ColorAxesData * data = new ColorAxesData(m_image);

  // Have to pass the data twice, because the second such statement
  // does not know about the precise implementation and extra
  // functions of this class. But it is the second statement
  // which will manage the memory.
  m_plotter = new ColorAxesPlotter(data);
  m_plotter->setData(data);

  // Use system specific thread count
  m_plotter->setRenderThreadCount(0);

  // Parse and set the colormap
  std::map<float, vw::cm::Vector3u> lut_map;
  try {
    vw::cm::parse_color_style(m_image.colormap, lut_map);
  } catch (...) {
    popUp("Unknown colormap style: " + m_image.colormap);
    m_image.colormap = "binary-red-blue";
    vw::cm::parse_color_style(m_image.colormap, lut_map);
  }
  m_plotter->setColorMap(new LutColormap(lut_map));
  
  // A color bar on the right axis. Must create a new colormap object
  // with the same data, to avoid a crash if using the same one.
  QwtScaleWidget *rightAxis = axisWidget(QwtPlot::yRight);
  QwtInterval zInterval = m_plotter->data()->interval(Qt::ZAxis);
  //rightAxis->setTitle("Intensity");
  rightAxis->setColorBarEnabled(true);
  rightAxis->setColorMap(zInterval, new LutColormap(lut_map));
  rightAxis->setColorBarWidth(30);

  m_plotter->setCachePolicy(QwtPlotRasterItem::PaintCache);
  m_plotter->attach(this);

  // TODO(oalexan1): Disable auto-scaling but ensure the colorbar is
  // next to the plots rather than leaving a large gap
  
  setAxisScale(QwtPlot::yRight, zInterval.minValue(), zInterval.maxValue());
  enableAxis(QwtPlot::yRight);
  setAxisAutoScale(QwtPlot::yRight, true);

  QwtScaleWidget * bottomAxis = axisWidget(QwtPlot::xBottom);
  setAxisAutoScale(QwtPlot::xBottom, true);
    
  // TODO(oalexan1): What does this do?
  plotLayout()->setAlignCanvasToScales(true);

  // Show it in image mode, not contour mode
  m_plotter->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);
  
  replot();
  
  // LeftButton for the zooming
  // MidButton for the panning
  // RightButton: zoom out by 1
  // Ctrl+RighButton: zoom out to full size
  
  QwtPlotZoomer* zoomer = new ColorAxesZoomer(canvas());
  zoomer->setMousePattern(QwtEventPattern::MouseSelect2,
                          Qt::RightButton, Qt::ControlModifier);
  zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
                          Qt::RightButton);
  
  const QColor c(Qt::darkBlue);
  zoomer->setRubberBandPen(c);
  zoomer->setTrackerPen(c);

  QwtPlotPanner *panner = new QwtPlotPanner(canvas());
  panner->setAxisEnabled(QwtPlot::yRight, false);
  panner->setMouseButton(Qt::MidButton);
  
  // Avoid jumping when labels with more/less digits
  // appear/disappear when scrolling vertically
  
  const QFontMetrics fm(axisWidget(QwtPlot::yLeft)->font());
  QwtScaleDraw *sd = axisScaleDraw(QwtPlot::yLeft);
  sd->setMinimumExtent(fm.width("100.00"));
  
}

}} // end namespace vw::gui

