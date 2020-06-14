
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_TRIGONOMETRY_H

#ifndef EXT_H_
#define EXT_H_

  #define SCALE ( 1 << 16 )

  static FT_Fixed
  square_root( FT_Fixed  val )
  {
    FT_ULong t, q, b, r;


    r = val;
    b = 0x40000000;
    q = 0;
    while( b > 0x40 )
    {
      t = q + b;
      if( r >= t )
      {
        r -= t;
        q = t + b; // equivalent to q += 2*b
      }
      r <<= 1;
      b >>= 1;
    }
    q >>= 8;

    return q;
  }

  static FT_Fixed
  cube_root( FT_Fixed val )
  {
    FT_Int v, g, c;


    if ( val == 0 || val == -SCALE || val == SCALE )
      return val;

    v = val < 0 ? -val : val;
    g = square_root( square_root( v ) );
    c = 0;

    while ( 1 )
    {
      c = FT_MulFix( FT_MulFix( g, g ), g ) - v;
      c = FT_DivFix( c, 3 * FT_MulFix( g, g ) );

      g -= c;

      if ( ( c < 0 ? -c : c ) < 30 )
        break;
    }

    return val < 0 ? -g : g;
  }

  static FT_Fixed
  arc_cos( FT_Fixed  val )
  {
    FT_Fixed  p, b = val;


    if ( b >  SCALE ) b =  SCALE;
    if ( b < -SCALE ) b = -SCALE;

    p = SCALE - FT_MulFix( b, b );
    p = square_root( p );

    return FT_Atan2( b, p );
  }

#endif /* EXT_H_ */

/* END */
