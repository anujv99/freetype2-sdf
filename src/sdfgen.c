
#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>
#include <stdlib.h>

#include <ft2build.h>
#include FT_INTERNAL_DEBUG_H

#include FT_FREETYPE_H
#include FT_ERRORS_H
#include FT_INTERNAL_OBJECTS_H

#include "sdfgen.h"

#define FT_26DOT6_TO_FLOAT( x )  ( (SDF_DataType)x / 64.0f )

#define EPSILON 0.000001f

  /* used to initialize vectors */
  static
  const SDF_Vector  zero_vector = { 0.0f, 0.0f };

  static SDF_Vector
  FT_To_SDF_Vec( FT_Vector  vec )
  {
    SDF_Vector  out_vec;


    out_vec.x = FT_26DOT6_TO_FLOAT( vec.x );
    out_vec.y = FT_26DOT6_TO_FLOAT( vec.y );
    return out_vec;
  }

  /* the process for generating signed distance field from       */
  /* outlines is based on the thesis:                            */
  /* Chlumsky, Viktor.Shape Decomposition for Multi-channel      */
  /* Distance Fields.Master's thesis. Czech Technical University */
  /* in Prague, Faculty of InformationTechnology, 2015.          */
  /* link: https://github.com/Chlumsky/msdfgen                   */

  FT_EXPORT_DEF( FT_Error )
  Generate_SDF( FT_Library     library,
                FT_GlyphSlot   glyph,
                FT_Bitmap     *abitmap )
  {
    SDF_Shape  shape;
    FT_Error   error   = FT_Err_Ok;

    FT_UInt    width   = 0u;
    FT_UInt    height  = 0u;
    FT_BBox    cBox;

    FT_Int     x_shift = 0;
    FT_Int     y_shift = 0;

    FT_Int     x_pad   = 0;
    FT_Int     y_pad   = 0;


    if ( !library )
      return FT_THROW( Invalid_Library_Handle );

    if ( !glyph || !abitmap || !glyph->face )
      return FT_THROW( Invalid_Argument );

    //if ( glyph->format != FT_GLYPH_FORMAT_OUTLINE )
    //  return FT_THROW( Invalid_Glyph_Format );

    /* compute the width and height and add padding */
    FT_Outline_Get_CBox( &glyph->outline, &cBox );

    width = abs( cBox.xMax - cBox.xMin ) / 64;
    height = abs( cBox.yMax - cBox.yMin ) / 64;

    x_pad = width / 4;
    y_pad = height / 4;

    width += x_pad;
    height += y_pad;

    x_shift = glyph->bitmap_left - x_pad / 2;
    y_shift = glyph->bitmap_top - glyph->bitmap.rows - y_pad / 2;

    /* align the outlne to the grid */
    FT_Outline_Translate(&glyph->outline, -x_shift * 64, -y_shift * 64);

    SDF_Shape_Init( &shape );
    shape.memory = library->memory;

    /* decompose the outline and store in into the SDF_Shape struct */
    /* this provide an easier way to iterate through all the curves */
    error = SDF_Decompose_Outline( &glyph->outline, &shape );
    if ( error != FT_Err_Ok )
      goto Exit;

    /* now loop through all the pixels and determine the shortest   */
    /* distance from the pixel's position to the nearest edge       */
    {
      FT_Memory  memory       = library->memory;
      FT_UInt    i            = 0u;
      FT_UInt    j            = 0u;

      SDF_DataType*     temp_buffer  = NULL;
      SDF_DataType      max_udist    = 0.0f; /* used to normalize values   */


      FT_MEM_ALLOC( temp_buffer, width * height * sizeof( SDF_DataType ) );

      for ( j = 0; j < height; j++ )
      {
        for ( i = 0; i < width; i++ )
        {
          SDF_Edge*             head;        /* used to iterate            */
          SDF_Vector            current_pos; /* current pixel position     */
          SDF_Signed_Distance   min_dist;    /* shortest signed distance   */
          SDF_DataType          min_udist;   /* shortest unsigned distance */


          head                    = shape.head;
          current_pos.x           = ( SDF_DataType )i;
          current_pos.y           = ( SDF_DataType )height -
                                    ( SDF_DataType )j;
          min_udist               = FLT_MAX;
          min_dist.direction      = zero_vector;
          min_dist.nearest_point  = zero_vector;

          /* currently we use a brute force algorithm to compute */
          /* shortest distance from all the curves               */
          while ( head != NULL )
          {
            SDF_Signed_Distance  dist;
            SDF_DataType         udist;


            error = get_min_distance( head, current_pos, &dist );
            if ( error != FT_Err_Ok )
              goto Exit;

            udist = sdf_vector_length( 
                      sdf_vector_sub( dist.nearest_point, current_pos ) );

            if ( fabs( min_udist ) - fabs( udist ) > EPSILON )
            {
              /* the two distances are fairly apart */
              min_udist = udist;
              min_dist = dist;
            }
            else
            {
              /* the two distances are pretty close, in this case */
              /* the endpoints might be connected and that is the */
              /* shortest distance                                */
              SDF_Vector  temp          = zero_vector;
              SDF_Vector  p_to_c        = zero_vector; /* point to curve */

              SDF_DataType  dir1        = 0.0f;
              SDF_DataType  dir2        = 0.0f;
              SDF_DataType  ortho1      = 0.0f;
              SDF_DataType  ortho2      = 0.0f;


              temp = sdf_vector_sub( dist.nearest_point,
                                     min_dist.nearest_point );
              if ( sdf_vector_length( temp ) <= EPSILON )
              {
                /* the nearest point is the endpoint and since two    */
                /* enpoint are connected we are getting two distance. */
                /* but both curves might have opposite direction, so  */
                /* now we have to determine the correct direction     */

                /* first check if the two directions are opposite     */
                p_to_c = sdf_vector_sub( dist.nearest_point, current_pos );

                dir1 = sdf_vector_cross( p_to_c, dist.direction );
                dir2 = sdf_vector_cross( p_to_c, min_dist.direction );

                if ( dir1 * dir2 <= 0.0f )
                {
                  /* the two directions are opposite so simply take   */
                  /* one with higher orthogonality                    */
                  p_to_c = sdf_vector_normalize( p_to_c );

                  ortho1 = sdf_vector_cross( dist.direction, p_to_c );
                  ortho2 = sdf_vector_cross( min_dist.direction, p_to_c );

                  ortho1 = fabs( ortho1 );
                  ortho2 = fabs( ortho2 );

                  /* now take the signed distance which makes same */
                  /* directions as `dir3'                          */
                  if ( ortho1 > ortho2 )
                  {
                    min_udist = udist;
                    min_dist = dist;
                  }
                }
              }
            }

            head = head->next;
          }

          if ( min_udist > max_udist )
            max_udist = min_udist;

          /* determine the sign */
          if ( sdf_vector_cross( 
                 sdf_vector_sub( min_dist.nearest_point, current_pos ),
                 min_dist.direction ) > 0)
            temp_buffer[j * width + i] = -0.0f; /* outside */
          else
            temp_buffer[j * width + i] = 1.0f;  /* inside  */
        }
      }

      /* release the previous buffer */
      FT_Bitmap_Done( library, abitmap );

      abitmap->width       = width;
      abitmap->rows        = height;
      abitmap->pitch       = width * sizeof( SDF_DataType );
      abitmap->num_grays   = 256;
      abitmap->pixel_mode  = 0; /* 32 bit floating point */

      /* normalize the values and put in the buffer */
      for ( i = 0; i < width * height; i++ )
      {
        //temp_buffer[i] = ( temp_buffer[i] ) / max_udist;
        //temp_buffer[i] += 1.0f;
        //temp_buffer[i] /= 2.0f;
      }

      abitmap->buffer = ( unsigned char* )temp_buffer;
    }

    Exit:
    FT_Outline_Translate(&glyph->outline, x_shift * 64, y_shift * 64);
    SDF_Shape_Done( &shape );
    return error;
  }


  /**************************************************************************
   *
   * functions.
   *
   */

  static
  const SDF_Edge  null_sdf_edge = { { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                    { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                    SDF_EDGE_TYPE_NONE, NULL };

  static
  const SDF_Shape  null_sdf_shape     = { { 0.0f, 0.0f }, NULL, 0u };

  FT_LOCAL_DEF( void )
  SDF_Edge_Init( SDF_Edge  *edge )
  {
    if ( edge )
      *edge = null_sdf_edge;
  }

  FT_LOCAL_DEF( void )
  SDF_Shape_Init( SDF_Shape  *shape )
  {
    if ( shape )
      *shape = null_sdf_shape;
  }

  FT_LOCAL_DEF( FT_Error )
  SDF_Shape_Done( SDF_Shape  *shape )
  {
    FT_Memory     memory;
    SDF_Edge*     head;

    if ( !shape )
      return FT_THROW( Invalid_Argument );

    if ( !shape->memory )
      return FT_THROW( Invalid_Library_Handle );

    memory = shape->memory;
    head = shape->head;

    while ( head )
    {
      SDF_Edge*  temp = head;


      head = head->next;
      FT_MEM_FREE( temp );
    }

    *shape = null_sdf_shape;

    return FT_Err_Ok;
  }
  
  /* functions for FT_Outline_Decompose */

  static int
  sdf_outline_move_to( const FT_Vector*  to,
                       void*             user )
  {
    SDF_Shape*  shape = ( SDF_Shape* )user;
    FT_Error    error = FT_Err_Ok;


    shape->current_pos = FT_To_SDF_Vec( *to );
    return error;
  }

  static int
  sdf_outline_line_to( const FT_Vector*  to,
                       void*             user )
  {
    SDF_Shape*    shape     = ( SDF_Shape* )user;
    SDF_Vector    endpoint  = FT_To_SDF_Vec( *to );
    SDF_Edge*     edge      = NULL;

    FT_Memory   memory      = shape->memory;
    FT_Error    error       = FT_Err_Ok;


    if ( endpoint.x != shape->current_pos.x ||
         endpoint.y != shape->current_pos.y )
    {
      /* only add edge if current_pos != to */
      FT_MEM_QNEW( edge );
      if (error != FT_Err_Ok)
        return error;
      SDF_Edge_Init( edge );
      
      edge->start_pos     = shape->current_pos;
      edge->end_pos       = endpoint;
      edge->edge_type     = SDF_EDGE_TYPE_LINE;

      /* add the edge to shape */
      if ( shape->head )
        edge->next = shape->head;

      shape->head         = edge;
      shape->current_pos  = endpoint;
      shape->num_edges    += 1u;
    }

    return error;
  }

  static int
  sdf_outline_conic_to( const FT_Vector*  control1,
                        const FT_Vector*  to,
                        void*             user )
  {
    SDF_Shape*    shape     = ( SDF_Shape* )user;
    SDF_Vector    endpoint  = FT_To_SDF_Vec( *to );
    SDF_Vector    control   = FT_To_SDF_Vec( *control1 );
    SDF_Edge*     edge      = NULL;

    FT_Memory     memory    = shape->memory;
    FT_Error      error     = FT_Err_Ok;


    /* in conic bezier, to and current_pos can be same */
    FT_MEM_QNEW( edge );
    if (error != FT_Err_Ok)
        return error;
    SDF_Edge_Init( edge );

    edge->start_pos        = shape->current_pos;
    edge->end_pos          = endpoint;
    edge->control_point_a  = control;
    edge->edge_type        = SDF_EDGE_TYPE_QUADRATIC_BEZIER;

    /* add the edge to shape */
    if ( shape->head )
      edge->next = shape->head;

    shape->head         = edge;
    shape->current_pos  = endpoint;
    shape->num_edges    += 1u;

    return error;
  }

  static int
  sdf_outline_cubic_to( const FT_Vector*  control1,
                        const FT_Vector*  control2,
                        const FT_Vector*  to,
                        void*             user )
  {
    SDF_Shape*    shape      = ( SDF_Shape* )user;
    SDF_Vector    endpoint   = FT_To_SDF_Vec( *to );
    SDF_Vector    control_a  = FT_To_SDF_Vec( *control1 );
    SDF_Vector    control_b  = FT_To_SDF_Vec( *control2 );
    SDF_Edge*     edge       = NULL;

    FT_Memory     memory     = shape->memory;
    FT_Error      error      = FT_Err_Ok;


    /* in cubic bezier, to and current_pos can be same */
    
    FT_MEM_QNEW( edge );
    if (error != FT_Err_Ok)
        return error;
    SDF_Edge_Init( edge );

    edge->start_pos        = shape->current_pos;
    edge->end_pos          = endpoint;
    edge->control_point_a  = control_a;
    edge->control_point_b  = control_b;
    edge->edge_type        = SDF_EDGE_TYPE_CUBIC_BEZIER;

    /* add the edge to shape */
    if ( shape->head )
      edge->next = shape->head;

    shape->head         = edge;
    shape->current_pos  = endpoint;
    shape->num_edges    += 1u;

    return error;
  }

  FT_LOCAL_DEF( FT_Error )
  SDF_Decompose_Outline( FT_Outline*  outline,
                         SDF_Shape   *shape )
  {
    /* initialize the FT_Outline_Funcs struct */
    FT_Outline_Funcs outline_decompost_funcs;
    outline_decompost_funcs.shift     = 0;
    outline_decompost_funcs.delta     = 0l;
    outline_decompost_funcs.move_to   = sdf_outline_move_to;
    outline_decompost_funcs.line_to   = sdf_outline_line_to;
    outline_decompost_funcs.conic_to  = sdf_outline_conic_to;
    outline_decompost_funcs.cubic_to  = sdf_outline_cubic_to;


    return FT_Outline_Decompose( outline, 
                                 &outline_decompost_funcs,
                                 ( void * )shape );
  }

  /**************************************************************************
   *
   * Math functions
   *
   */

  FT_LOCAL_DEF( SDF_DataType )
  clamp( SDF_DataType   input,
         SDF_DataType   min,
         SDF_DataType   max )
  {
    SDF_DataType  output = input;


    if ( output < min )
      output = min;
    if ( output > max )
      output =  max;

    return output;
  }

  FT_LOCAL_DEF( FT_UShort )
  solve_quadratic_equation( SDF_DataType  a,
                            SDF_DataType  b,
                            SDF_DataType  c,
                            SDF_DataType  out[2] )
  {
    SDF_DataType  discriminant       = 0.0f;
    SDF_DataType  discriminant_root  = 0.0f;


    /* if a == 0.0f then the equation is linear */
    if ( a == 0.0f )
    {
      /* if b == 0.0f then c == 0 is false */
      if ( b == 0.0f )
      {
        return 0;
      }
      else
      {
        out[0] = -c / b;
        return 1;
      }
    }

    /* compute the discriminant ( i.e. b^2 - 4ac ) */
    discriminant = ( b * b ) - ( 4.0f * a * c );

    if ( discriminant < 0.0f )
    {
      /* both unreal comlex roots */
      return 0;
    }
    else if ( discriminant == 0.0f )
    {
      /* equal real roots */
      /* ( -b ) / 2a */
      out[0] = -b / ( 2 * a );

      return 1;
    }
    else /* discriminant > 0.0f */
    {
      /* equal real roots */
      /* ( -b + discriminant^0.5 ) / 2a */
      /* ( -b - discriminant^0.5 ) / 2a */
      discriminant_root = sqrtf( discriminant );
      out[0] = ( -b + discriminant_root ) / ( 2 * a );
      out[1] = ( -b - discriminant_root ) / ( 2 * a );

      return 2;
    }
  }

  FT_LOCAL_DEF( FT_UShort )
  solve_cubic_equation( SDF_DataType a,
                        SDF_DataType b,
                        SDF_DataType c,
                        SDF_DataType d,
                        SDF_DataType out[3] )
  {
    /* the method here is a direct implementation of the  */
    /* `Cardano's Cubic formula' given here               */
    /* https://mathworld.wolfram.com/CubicFormula.html    */

    SDF_DataType  q              = 0.0f;  /* intermediate      */
    SDF_DataType  r              = 0.0f;  /* intermediate      */
    SDF_DataType  discriminant   = 0.0f;  /* discriminant      */

    SDF_DataType  a2             = b;     /* x^2 coefficients  */
    SDF_DataType  a1             = c;     /* x coefficients    */
    SDF_DataType  a0             = d;     /* constant          */

    if ( a == 0 )
    {
      /* quadratic equation */
      return solve_quadratic_equation( b, c, d, out );
    }

    if ( fabs(a) != 1.0f)
    {
      /* normalize the coefficients */
      a2 /= a;
      a1 /= a;
      a0 /= a;
    }

    q = ( ( 3 * a1 ) - ( a2 * a2 ) ) / 9.0f;
    r = ( ( 9 * a1 * a2) - ( 27 * a0 ) - ( 2 * a2 * a2 * a2) ) / 54;
    discriminant = ( q * q * q ) + ( r * r );

    if ( discriminant < 0.0f )
    { 
      SDF_DataType  t = 0.0f;  /* angle theta */


      /* all real unequal roots */
      t = acos( r / sqrtf( -( q * q * q ) ) );
      a2 /= 3.0f;
      q = sqrtf( -q );
      out[0] = 2 * q * cos( t / 3.0f ) - a2;
      out[1] = 2 * q * cos( ( t + 2 * M_PI ) / 3.0f ) - a2;
      out[2] = 2 * q * cos( ( t + 4 * M_PI ) / 3.0f ) - a2;

      return 3;
    }
    else if ( discriminant == 0.0f )
    {
      SDF_DataType  s = 0.0f;  /* intermediate */


      /* all real roots and at least two are equal */
      s = cbrtf( r );
      a2 /= -3.0f;
      out[0] = a2 + ( 2 * s );
      out[1] = a2 - s;

      return 2;
    }
    else /* discriminant > 0.0f */
    {
      SDF_DataType  s = 0.0f;  /* intermediate */
      SDF_DataType  t = 0.0f;  /* intermediate */


      /* only one real root */
      discriminant = sqrtf( discriminant );
      s = cbrtf( r + discriminant );
      t = cbrtf( r - discriminant );
      a2 /= -3.0f;
      out[0] = a2 + ( s + t );

      return 1;
    }

  }

  FT_LOCAL_DEF( SDF_DataType )
  sdf_vector_length( SDF_Vector  vector )
  {
    return sqrtf( ( vector.x * vector.x ) + ( vector.y * vector.y ) );
  }

  FT_LOCAL_DEF( SDF_DataType )
  sdf_vector_squared_length( SDF_Vector  vector )
  {
    return ( ( vector.x * vector.x ) + ( vector.y * vector.y ) );
  }

  FT_LOCAL_DEF( SDF_Vector )
  sdf_vector_add( SDF_Vector  a,
                  SDF_Vector  b )
  {
    SDF_Vector  output;

    
    output.x = a.x + b.x;
    output.y = a.y + b.y;

    return output;
  }

  FT_LOCAL_DEF( SDF_Vector )
  sdf_vector_sub( SDF_Vector  a,
                  SDF_Vector  b )
  {
    SDF_Vector  output;

    
    output.x = a.x - b.x;
    output.y = a.y - b.y;

    return output;
  }

  FT_LOCAL_DEF( SDF_Vector )
  sdf_vector_normalize( SDF_Vector  vector )
  {
    SDF_Vector    output;
    SDF_DataType  vector_length = sdf_vector_length( vector );


    output.x = vector.x / vector_length;
    output.y = vector.y / vector_length;
    
    return output;
  }

  FT_LOCAL_DEF( SDF_Vector )
  sdf_vector_scale( SDF_Vector    vector,
                    SDF_DataType  scale )
  {
    SDF_Vector  output;


    output.x = vector.x * scale;
    output.y = vector.y * scale;

    return output;
  }

  FT_LOCAL_DEF( SDF_DataType )
  sdf_vector_dot( SDF_Vector  a, 
                  SDF_Vector  b )
  {
    return ( ( a.x * b.x ) + ( a.y * b.y ) );
  }

  FT_LOCAL_DEF( SDF_DataType )
  sdf_vector_cross( SDF_Vector  a,
                    SDF_Vector  b )
  {
    return ( ( a.x * b.y ) - ( a.y * b.x ) );
  }

  FT_LOCAL_DEF( FT_Bool )
  sdf_vector_equal( SDF_Vector a,
                    SDF_Vector b )
  {
    /* probably should use EPSILON */
    if ( a.x == b.x && a.y == b.y )
        return 1;
    return 0;
  }

  FT_LOCAL_DEF( FT_Error )
  get_min_distance( SDF_Edge*             edge,
                    const SDF_Vector      point,
                    SDF_Signed_Distance  *out )
  {
    /* compute shortest distance from `point' to the `edge' */

    FT_Error  error = FT_Err_Ok;


    if ( !edge || !out )
      return FT_THROW( Invalid_Argument );

    switch ( edge->edge_type ) {
    case SDF_EDGE_TYPE_LINE:
    {
      /* in order to calculate the shortest distance from a point to */
      /* a line segment.                                             */
      /*                                                             */
      /* a = start point of the line segment                         */
      /* b = end point of the line segment                           */
      /* p = point from which shortest distance is to be calculated  */
      /* ----------------------------------------------------------- */
      /* => we first write the parametric equation of the line       */
      /*    point_on_line = a + ( b - a ) * t ( t is the factor )    */
      /*                                                             */
      /* => next we find the projection of point p on the line. the  */
      /*    projection will be perpendicular to the line, that is    */
      /*    why we can find it by making the dot product zero.       */
      /*    ( point_on_line - a ) . ( p - point_on_line ) = 0        */
      /*                                                             */
      /*                 ( point_on_line )                           */
      /*    ( a ) x-------o----------------x ( b )                   */
      /*                |_|                                          */
      /*                  |                                          */
      /*                  |                                          */
      /*                ( p )                                        */
      /*                                                             */
      /* => by simplifying the above equation we get the factor of   */
      /*    point_on_line such that                                  */
      /*    t = ( ( p - a ) . ( b - a ) ) / ( |b - a| ^ 2 )          */
      /*                                                             */
      /* => we clamp the factor t between [0.0f, 1.0f], because the  */
      /*    point_on_line can be outside the line segment.           */
      /*                                                             */
      /*                                        ( point_on_line )    */
      /*    ( a ) x------------------------x ( b ) -----o---         */
      /*                                              |_|            */
      /*                                                |            */
      /*                                                |            */
      /*                                              ( p )          */
      /*                                                             */
      /* => finally the distance becomes | point_on_line - p |       */

      const SDF_Vector  a                       = edge->start_pos;
      const SDF_Vector  b                       = edge->end_pos;
      const SDF_Vector  p                       = point;
      const SDF_Vector  line_segment            = sdf_vector_sub( b, a );
      const SDF_Vector  p_sub_a                 = sdf_vector_sub( p, a );

      SDF_Vector        nearest_point           = zero_vector;

      SDF_DataType      segment_squared_length  = 0.0f;
      SDF_DataType      factor                  = 0.0f;
      SDF_DataType      sign                    = 0.0f;


      /* if both the endpoints of the line segmnet coincide then the   */
      /* shortest distance will be from `point' to any of the endpoint */
      if ( sdf_vector_equal( a, b ) == 1 )
      {
        out->nearest_point = a;
        out->direction = sdf_vector_normalize( line_segment );
        break;
      }

      factor = sdf_vector_dot( p_sub_a, line_segment ) /
               sdf_vector_squared_length( line_segment );
      factor = clamp( factor, 0.0f, 1.0f );

      nearest_point = sdf_vector_scale( line_segment, factor );
      nearest_point = sdf_vector_add( a, nearest_point );

      out->nearest_point = nearest_point;
      out->direction = sdf_vector_normalize( line_segment );

      break;
    }
    case SDF_EDGE_TYPE_QUADRATIC_BEZIER:
    {
      /* the procedure to find the shortest distance from a point to */
      /* a quadratic bezier curve is simliar to a line segment. the  */
      /* shortest distance will be perpendicular to the bezier curve */
      /* The only difference from line is that there can be more     */
      /* than one perpendicular and we also have to check the endpo- */
      /* -ints, because the perpendicular may not be the shortest.   */
      /*                                                             */
      /* p0 = first endpoint                                         */
      /* p1 = control point                                          */
      /* p2 = second endpoint                                        */
      /* p = point from which shortest distance is to be calculated  */
      /* ----------------------------------------------------------- */
      /* => the equation of a quadratic bezier curve can be written  */
      /*    B( t ) = ( ( 1 - t )^2 )p0 + 2( 1 - t )tp1 + t^2p2       */
      /*    here t is the factor with range [0.0f, 1.0f]             */
      /*    the above equation can be rewritten as                   */
      /*    B( t ) = t^2( p0 - 2p1 + p2 ) + 2t( p1 - p0 ) + p0       */
      /*                                                             */
      /*    now let A = ( p0 - 2p1 + p2), B = ( p1 - p0 )            */
      /*    B( t ) = t^2( A ) + 2t( B ) + p0                         */
      /*                                                             */
      /* => the derivative of the above equation is written as       */
      /*    B`( t ) = 2( tA + B )                                    */
      /*                                                             */
      /* => now to find the shortest distance from p to B( t ), we   */
      /*    find the point on the curve at which the shortest        */
      /*    distance vector ( i.e. B( t ) - p ) and the direction    */
      /*    ( i.e. B`( t )) makes 90 degrees. in other words we make */
      /*    the dot product zero.                                    */
      /*    ( B( t ) - p ).( B`( t ) ) = 0                           */
      /*    ( t^2( A ) + 2t( B ) + p0 - p ).( 2( tA + B ) ) = 0      */
      /*                                                             */
      /*    after simplifying we get a cubic equation as             */
      /*    at^3 + bt^2 + ct + d = 0                                 */
      /*    a = ( A.A ), b = ( 3A.B ), c = ( 2B.B + A.p0 - A.p )     */
      /*    d = ( p0.B - p.B )                                       */
      /*                                                             */
      /* => now the roots of the equation can be computed using the  */
      /*    `Cardano's Cubic formula'                                */
      /*    ( https://mathworld.wolfram.com/CubicFormula.html )      */
      /*    we discard the roots which do not lie in the range       */
      /*    [0.0f, 1.0f] and also check the endpoints ( p0, p2 )     */
      /*                                                             */
      /* [note]: B and B( t ) are different in the above equations   */

      SDF_Vector    aA             = zero_vector; /* A in above comment */
      SDF_Vector    bB             = zero_vector; /* B in above comment */
      SDF_Vector    nearest_point  = zero_vector;
      SDF_Vector    temp           = zero_vector;
                    
      SDF_Vector    p0             = edge->start_pos;
      SDF_Vector    p1             = edge->control_point_a;
      SDF_Vector    p2             = edge->end_pos;
      SDF_Vector    p              = point;

      /* cubic coefficients */
      SDF_DataType  a         = 0.0f;
      SDF_DataType  b         = 0.0f;
      SDF_DataType  c         = 0.0f;
      SDF_DataType  d         = 0.0f;
      SDF_DataType  min       = FLT_MAX; /* shortest distace */
                            
      SDF_DataType  roots[5]  = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

      /* factor `t' of the shortest point on curve */
      SDF_DataType  np_factor = 0.0f;

      FT_UShort     num_r     = 0;   /* number of roots */
      FT_UShort     i         = 0;


      aA = sdf_vector_add( p0, p2 );
      aA = sdf_vector_sub( aA, p1 );
      aA = sdf_vector_sub( aA, p1 );

      bB = sdf_vector_sub( p1, p0 );

      a  = sdf_vector_dot( aA, aA );
      b  = sdf_vector_dot( aA, bB );
      b *= 3.0f;
      c  = sdf_vector_dot( bB, bB );
      c *= 2.0f;
      c += sdf_vector_dot( aA, p0 ) -
           sdf_vector_dot( aA, p );
      d  = sdf_vector_dot( p0, bB ) -
           sdf_vector_dot( p, bB );

      num_r = solve_cubic_equation( a, b, c, d, roots + 2 );

      /* this will take care of the endpoints */
      roots[0] = 0.0f;
      roots[1] = 1.0f;
      num_r += 2;

      for ( i = 0; i < num_r; i++ )
      {
        SDF_DataType  t             = roots[i];
        SDF_DataType  t2            = t * t;
        SDF_DataType  dist          = 0.0f;
        
        SDF_Vector curve_point      = zero_vector;  /* point on the curve */


        /* only check of t in range [0.0f, 1.0f] */
        if ( t < 0.0f || t > 1.0f )
          continue;

        /* B( t ) = t^2( A ) + 2t( B ) + p0  */
        curve_point  = sdf_vector_scale( aA, t2 );
        temp = sdf_vector_scale( bB, 2.0f * t );
        curve_point = sdf_vector_add( curve_point, temp );
        curve_point = sdf_vector_add( curve_point, p0 );

        /* compute distance from `point' to the `curve_point' */
        temp = sdf_vector_sub( curve_point, p );
        dist = sdf_vector_length( temp );

        if ( dist < min )
        {
          min = dist;
          nearest_point = curve_point;
          np_factor = t;
        }
      }

      out->nearest_point = nearest_point;
      
      /* calculate direction of shortest point on the curve using  */
      /* B`( t ) = 2( tA + B )                                     */
      temp = sdf_vector_scale( aA, np_factor );
      temp = sdf_vector_add( temp, bB );
      temp = sdf_vector_scale( temp, 2.0f );
      temp = sdf_vector_normalize( temp );
      out->direction = temp;
      break;
    }
    case SDF_EDGE_TYPE_CUBIC_BEZIER:
    {
      /* the procedure to find the shortest distance from a point to */
      /* a cubic bezier curve is simliar to a quadratic curve.       */
      /* The only difference is that while calculating the factor    */
      /* `t', instead of a cubic polynomial equation we have to find */
      /* the roots of a 5th degree polynomial equation.              */
      /* But since solving a 5th degree polynomial equation require  */
      /* significant amount of time and still the results may not be */
      /* accurate, we are going to directly approximate the value of */
      /* `t' using Newton-Raphson method                             */
      /*                                                             */
      /* p0 = first endpoint                                         */
      /* p1 = first control point                                    */
      /* p2 = seconf control point                                   */
      /* p3 = second endpoint                                        */
      /* p = point from which shortest distance is to be calculated  */
      /* ----------------------------------------------------------- */
      /* => the equation of a cubic bezier curve can be written as:  */
      /*    B( t ) = ( ( 1 - t )^3 )p0 + 3( ( 1 - t )^2 )tp1 +       */
      /*             3( 1 - t )( t^2 )p2 + ( t^3 )p3                 */
      /*    The equation can be expanded and written as:             */
      /*    B( t ) = ( t^3 )( -p0 + 3p1 - 3p2 + p3 ) +               */
      /*             3( t^2 )( p0 - 2p1 + p2 ) + 3t( -p0 + p1 ) + p0 */
      /*                                                             */
      /*    Now let A = ( -p0 + 3p1 - 3p2 + p3 ),                    */
      /*            B = 3( p0 - 2p1 + p2 ), C = 3( -p0 + p1 )        */
      /*    B( t ) = t^3( A ) + t^2( B ) + tC + p0                   */
      /*                                                             */
      /* => the derivative of the above equation is written as       */
      /*    B`( t ) = 3t^2( A ) + 2t( B ) + C                        */
      /*                                                             */
      /* => further derivative of the above equation is written as   */
      /*    B``( t ) = 6t( A ) + 2B                                  */
      /*                                                             */
      /* => the equation of distance from point `p' to the curve     */
      /*    P( t ) can be written as                                 */
      /*    P( t ) = t^3( A ) + t^2( B ) + tC + p0 - p               */
      /*    Now let D = ( p0 - p )                                   */
      /*    P( t ) = t^3( A ) + t^2( B ) + tC + D                    */
      /*                                                             */
      /* => finally the equation of angle between curve B( t ) and   */
      /*    point to curve distance P( t ) can be written as         */
      /*    Q( t ) = P( t ).B`( t )                                  */
      /*                                                             */
      /* => now our task is to find a value of t such that the above */
      /*    equation Q( t ) becomes zero. in other words the point   */
      /*    to curve vector makes 90 degree with curve. this is done */
      /*    by Newton-Raphson's method.                              */
      /*                                                             */
      /* => we first assume a arbitary value of the factor `t' and   */
      /*    then we improve it using Newton's equation such as       */
      /*                                                             */
      /*    t -= Q( t ) / Q`( t )                                    */
      /*    putting value of Q( t ) from the above equation gives    */
      /*                                                             */
      /*    t -= P( t ).B`( t ) / derivative( P( t ).B`( t ) )       */
      /*    t -= P( t ).B`( t ) /                                    */
      /*         ( P`( t )B`( t ) + P( t ).B``( t ) )                */
      /*                                                             */
      /*    P`( t ) is noting but B`( t ) because the constant are   */
      /*    gone due to derivative                                   */
      /*                                                             */
      /* => finally we get the equation to improve the factor as     */
      /*    t -= P( t ).B`( t ) /                                    */
      /*         ( B`( t ).B`( t ) + P( t ).B``( t ) )               */
      /*                                                             */
      /* [note]: B and B( t ) are different in the above equations   */

      SDF_Vector    aA             = zero_vector; /* A in above comment */
      SDF_Vector    bB             = zero_vector; /* B in above comment */
      SDF_Vector    cC             = zero_vector; /* C in above comment */
      SDF_Vector    dD             = zero_vector; /* D in above comment */
      SDF_Vector    nearest_point  = zero_vector;
      SDF_Vector    temp           = zero_vector;
                    
      SDF_Vector    p0             = edge->start_pos;
      SDF_Vector    p1             = edge->control_point_a;
      SDF_Vector    p2             = edge->control_point_b;
      SDF_Vector    p3             = edge->end_pos;
      SDF_Vector    p              = point;

      SDF_DataType  min_distance   = FLT_MAX;
      SDF_DataType  min_factor     = 0.0f;
      
      FT_UShort     iterations     = 0;
      FT_UShort     steps          = 0;

      const FT_UShort  MAX_STEPS      = 4;
      const FT_UShort  MAX_DIVISIONS  = 4;


      aA = sdf_vector_sub( p1, p2 );
      aA = sdf_vector_scale( aA, 3.0f );
      aA = sdf_vector_sub( aA, p0 );
      aA = sdf_vector_add( aA, p3 );

      bB = sdf_vector_scale( p1, -2.0f );
      bB = sdf_vector_add( bB, p0 );
      bB = sdf_vector_add( bB, p2 );
      bB = sdf_vector_scale( bB, 3.0f );

      cC = sdf_vector_sub( p1, p0 );
      cC = sdf_vector_scale( cC, 3.0f );

      dD = sdf_vector_sub( p0, p );

      for ( iterations = 0; iterations <= MAX_DIVISIONS; iterations++ )
      {
        SDF_DataType  factor  = ( SDF_DataType )iterations /
                                ( SDF_DataType )MAX_DIVISIONS;
        SDF_DataType  factor2 = 0.0f;
        SDF_DataType  factor3 = 0.0f;
        SDF_DataType  length  = 0.0f;

        SDF_Vector    point_to_curve  = zero_vector;  /* P( t ) above      */
        SDF_Vector    d1              = zero_vector;  /* first derivative  */
        SDF_Vector    d2              = zero_vector;  /* second derivative */


        for ( steps = 0; steps < MAX_STEPS; steps++ )
        {
          factor2 = factor * factor;
          factor3 = factor2 * factor;

          /* calculate P( t ) = t^3( A ) + t^2( B ) + tC + D  */
          point_to_curve = sdf_vector_scale( aA, factor3 );
          temp = sdf_vector_scale( bB, factor2 );
          point_to_curve = sdf_vector_add( point_to_curve, temp );
          temp = sdf_vector_scale( cC, factor );
          point_to_curve = sdf_vector_add( point_to_curve, temp );
          point_to_curve = sdf_vector_add( point_to_curve, dD );

          length = sdf_vector_length( point_to_curve );

          if ( length < min_distance )
          {
            min_distance = length;
            min_factor = factor;
            nearest_point = sdf_vector_add( point_to_curve, p );
          }

          /* calculate B`( t ) = 3t^2( A ) + 2t( B ) + C  */
          d1 = sdf_vector_scale( aA, 3.0f * factor2 );
          temp = sdf_vector_scale( bB, 2.0f * factor );
          d1 = sdf_vector_add( d1, temp );
          d1 = sdf_vector_add( d1, cC );

          /* calculate B``( t ) = 6t( A ) + 2B  */
          d2 = sdf_vector_scale( aA, 3.0f * factor );
          d2 = sdf_vector_add( d2, bB );
          d2 = sdf_vector_scale( d2, 2.0f );

          /* improve factor using                          */
          /*    t -= P( t ).B`( t ) /                      */
          /*         ( B`( t ).B`( t ) + P( t ).B``( t ) ) */
          factor -=   sdf_vector_dot( point_to_curve, d1 ) /
                    ( sdf_vector_dot( d1, d1 ) +
                      sdf_vector_dot( point_to_curve, d2 ) );

          if ( factor > 1.0f || factor < 0.0f )
          {
            break;
          }
        }
      }

      out->nearest_point = nearest_point;

      /* calculate direction of shortest point on the curve using  */
      /* B`( t ) = 3t^2( A ) + 2t( B ) + C                         */
      out->direction = sdf_vector_scale( aA, 
                                         3.0f * min_factor * min_factor );
      temp = sdf_vector_scale( bB, 2 * min_factor );
      out->direction = sdf_vector_add( out->direction, temp );
      out->direction = sdf_vector_add( out->direction, cC );
      out->direction = sdf_vector_normalize( out->direction );
      break;
    }
    default:
      error = FT_THROW( Invalid_Argument );
    }

    return error;
  }

/* END */
