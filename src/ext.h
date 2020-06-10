
#define _USE_MATH_DEFINES
#include <math.h>

#include <ft2build.h>
#include FT_FREETYPE_H

#ifndef EXT_H_
#define EXT_H_

  static FT_Fixed
  arc_cos( FT_Fixed  val )
  {
    double  temp    = ( double )val / 65536;
    double  arccos  = acos( temp );

    arccos *= 180 / M_PI;
    return ( FT_Fixed )( arccos * 65536 );
  }

  static FT_Fixed
  square_root( FT_Fixed val )
  {
    double  temp    = ( double )val / 65536;
    double  root    = sqrt( temp );

    return ( FT_Fixed )( root * 65536 );
  }

  static FT_Fixed
  cube_root( FT_Fixed val )
  {
    double  temp    = ( double )val / 65536;
    double  root    = cbrt( temp );

    return ( FT_Fixed )( root * 65536 );
  }

#endif /* EXT_H_ */

/* END */
