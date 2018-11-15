#include "render.h"

namespace kwiver {
namespace arrows{

double check_neighbouring_pixels_depth_map(const kwiver::vital::image& img, double x, double y)
{
  /// check the four closest pixels and return the average of the finite values
//  int x1 = static_cast<int>(x);
//  int y1 = static_cast<int>(y);
//   double v1;
//  double sum = 0.0;
//  int nb = 0;
//   if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  ++x1;
//  if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  ++y1;
//  if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  --x1;
//  if (x1 >= 0 && x1 < static_cast<int>(image.width()) && y1 >= 0 && y1 < static_cast<int>(image.height()))
//  {
//    v1 = image.at<double>(x1, y1);
//    if (!std::isinf(v1))
//    {
//      sum += v1;
//      nb++;
//    }
//  }
//  if (nb > 0)
//    return sum / nb;
//  else
//    return std::numeric_limits<double>::infinity();


//  if (x < 0 || y < 0 || x > img.width() - 1|| y > img.height() - 1)
//    return 0.0;

//  int p1x = static_cast<int>(x);
//  double normx = x - p1x;
//  int p1y = static_cast<int>(y);
//  double normy = y - p1y;

//  ptrdiff_t w_step = img.w_step(), h_step = img.h_step();
//  const double* pix1 = reinterpret_cast<const double*>(img.first_pixel()) + h_step * p1y + w_step * p1x;

//  if (normx < 1e-5 && normy < 1e-5) return *pix1;


//  if (normx < 1e-5)
//  {
//    if (!std::isinf(pix1[0]) && !std::isinf(pix1[h_step]))
//    {
//      return pix1[0] + (pix1[h_step] - pix1[0]) * normy;
//    }
//    else
//      return std::min(pix1[0], pix1[h_step]);
//  }
//  if (normy < 1e-5)
//  {
//    if (!std::isinf(pix1[0]) && !std::isinf(pix1[w_step]))
//    {
//      return pix1[0] + (pix1[w_step] - pix1[0]) * normx;
//    }
//    else
//      return std::min(pix1[0], pix1[w_step]);
//  }



//  double i1, i2;
//  if (!std::isinf(pix1[0]) && !std::isinf(pix1[h_step]))
//  {
//    i1 = pix1[0] + (pix1[h_step] - pix1[0]) * normy;
//  }
//  else
//    i1 = std::min(pix1[0], pix1[h_step]);

//  if (!std::isinf(pix1[w_step]) && !std::isinf(pix1[w_step + h_step]))
//  {
//    i2 = pix1[w_step] + (pix1[w_step + h_step] - pix1[w_step]) * normy;
//  }
//  else
//    i2 = std::min(pix1[w_step], pix1[w_step + h_step]);



//  if (!std::isinf(i1) && !std::isinf(i2))
//  {
//    return i1 + (i2 - i1) * normx;
//  }
//  else
//    return std::min(i1, i2);



  if (x < 0 || y < 0 || x > img.width() - 1|| y > img.height() - 1)
    return 0.0;


  int p1x = static_cast<int>(x);
  double normx = x - p1x;
  int p1y = static_cast<int>(y);
  double normy = y - p1y;

  ptrdiff_t w_step = img.w_step(), h_step = img.h_step();
  const double* pix1 = reinterpret_cast<const double*>(img.first_pixel()) + h_step * p1y + w_step * p1x;



  if (std::isinf(pix1[0]) || std::isinf(pix1[w_step]) || std::isinf(pix1[h_step]) || std::isinf(pix1[w_step+h_step]))
  {
    std::cout << "at least one is inf" << std::endl;
    std::cout << pix1[0] << " " << pix1[w_step] << " " << pix1[w_step+h_step] << " " << pix1[h_step] << std::endl;

    int nb = 0;
    double sum = 0.0;
    if (!std::isinf(pix1[0]))
    {
      sum += pix1[0];
      ++nb;
    }
    if (!std::isinf(pix1[w_step]))
    {
      sum += pix1[w_step];
      ++nb;
    }
    if (!std::isinf(pix1[h_step]))
    {
      sum += pix1[h_step];
      ++nb;
    }
    if (!std::isinf(pix1[w_step + h_step]))
    {
      sum += pix1[w_step + h_step];
      ++nb;
    }
    if (nb > 0)
    {
      std::cout << "ret " << sum/nb << std::endl;
      return sum / nb;
     }
    else
    {std::cout << "ret inf " << std::endl;
      return std::numeric_limits<double>::infinity();
    }
  }




  if (normx == 0 && normy == 0) return *pix1;
  if (normx == 0) return pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  if (normy == 0) return pix1[0] + (pix1[w_step] - pix1[0]) * normx;

  double i1 = pix1[0] + (pix1[h_step] - pix1[0]) * normy;
  double i2 = pix1[w_step] + (pix1[w_step + h_step] - pix1[w_step]) * normy;

  return i1 + (i2 - i1) * normx;
}

}
}
