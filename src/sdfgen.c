
#include <math.h>

#include <ft2build.h>
#include FT_INTERNAL_DEBUG_H

#include FT_FREETYPE_H
#include FT_ERRORS_H
#include FT_INTERNAL_OBJECTS_H

#include "sdfgen.h"

#define FT_26DOT6_TO_FLOAT( x )  ( (float)x / 64.0f )

  static SDF_Vector
  FT_To_SDF_Vec( FT_Vector  vec )
  {
    SDF_Vector  out_vec;


    out_vec.x = FT_26DOT6_TO_FLOAT( vec.x );
    out_vec.y = FT_26DOT6_TO_FLOAT( vec.y );
    return out_vec;
  }

  FT_EXPORT_DEF( FT_Error )
  FT_Generate_SDF( FT_Library   library,
                   FT_Outline*  outline,
                   FT_Bitmap   *abitmap )
  {
    SDF_Shape  shape;
    FT_Error   error = FT_Err_Ok;


    if ( !library )
      return FT_THROW( Invalid_Library_Handle );

    if ( !outline || !abitmap )
      return FT_THROW( Invalid_Argument );

    SDF_Shape_Init( &shape );
    shape.memory = library->memory;

    /* decompost the outline and store in into the SDF_Shape struct   */
    /* this provide an easier way to iterate through all the contours */
    error = SDF_Decompose_Outline( outline, &shape );
    if ( error != FT_Err_Ok )
      goto Exit;

    Exit:
    SDF_Shape_Done( &shape );
    return error;
  }


  /**************************************************************************
   *
   * functions.
   *
   */

  static
  const SDF_Vector zero_vector       = { 0.0f, 0.0f };

  static
  const SDF_Contour null_sdf_contour = { { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                         { 0.0f, 0.0f }, { 0.0f, 0.0f },
                                         SDF_CONTOUR_TYPE_NONE, NULL };

  static
  const SDF_Shape null_sdf_shape     = { { 0.0f, 0.0f }, NULL, 0u };

  FT_LOCAL_DEF( void )
  SDF_Contour_Init( SDF_Contour  *contour )
  {
    if ( contour )
      *contour = null_sdf_contour;
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
    SDF_Contour*  head;

    if ( !shape )
      return FT_THROW( Invalid_Argument );

    if ( !shape->memory )
      return FT_THROW( Invalid_Library_Handle );

    memory = shape->memory;
    head = shape->contour_head;

    while ( head )
    {
      SDF_Contour*  temp = head;


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
    SDF_Contour*  contour   = NULL;

    FT_Memory   memory      = shape->memory;
    FT_Error    error       = FT_Err_Ok;


    if ( endpoint.x != shape->current_pos.x ||
         endpoint.y != shape->current_pos.y )
    {
      /* only add contour if current_pos != to */
      FT_MEM_QNEW( contour );
      if (error != FT_Err_Ok)
        return error;
      SDF_Contour_Init( contour );
      
      contour->start_pos     = shape->current_pos;
      contour->end_pos       = endpoint;
      contour->contour_type  = SDF_CONTOUR_TYPE_LINE;

      /* add the contour to shape */
      if ( shape->contour_head )
        contour->next = shape->contour_head;

      shape->contour_head  = contour;
      shape->current_pos   = endpoint;
      shape->num_contours += 1u;
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
    SDF_Contour*  contour   = NULL;

    FT_Memory     memory    = shape->memory;
    FT_Error      error     = FT_Err_Ok;


    /* in conic bezier, to and current_pos can be same */
    FT_MEM_QNEW( contour );
    if (error != FT_Err_Ok)
        return error;
    SDF_Contour_Init( contour );

    contour->start_pos        = shape->current_pos;
    contour->end_pos          = endpoint;
    contour->control_point_a  = control;
    contour->contour_type     = SDF_CONTOUR_TYPE_QUADRATIC_BEZIER;

    /* add the contour to shape */
    if ( shape->contour_head )
      contour->next = shape->contour_head;

    shape->contour_head  = contour;
    shape->current_pos   = endpoint;
    shape->num_contours += 1u;

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
    SDF_Contour*  contour    = NULL;

    FT_Memory     memory     = shape->memory;
    FT_Error      error      = FT_Err_Ok;


    /* in cubic bezier, to and current_pos can be same */
    
    FT_MEM_QNEW( contour );
    if (error != FT_Err_Ok)
        return error;
    SDF_Contour_Init( contour );

    contour->start_pos        = shape->current_pos;
    contour->end_pos          = endpoint;
    contour->control_point_a  = control_a;
    contour->control_point_b  = control_b;
    contour->contour_type     = SDF_CONTOUR_TYPE_CUBIC_BEZIER;

    /* add the contour to shape */
    if ( shape->contour_head )
      contour->next = shape->contour_head;

    shape->contour_head  = contour;
    shape->current_pos   = endpoint;
    shape->num_contours += 1u;

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

  FT_LOCAL_DEF( float )
  clamp( float   input,
         float   min,
         float   max )
  {
    float  output = input;


    if ( output < min )
      output = min;
    if ( output > max )
      output =  max;

    return output;
  }

  FT_LOCAL_DEF( float )
  sdf_vector_length( SDF_Vector  vector )
  {
    return sqrtf( ( vector.x * vector.x ) + ( vector.y * vector.y ) );
  }

  FT_LOCAL_DEF( float )
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
    SDF_Vector  output;
    float       vector_length = sdf_vector_length( vector );


    output.x = vector.x / vector_length;
    output.y = vector.y / vector_length;
    
    return output;
  }

  FT_LOCAL_DEF( SDF_Vector )
  sdf_vector_scale( SDF_Vector  vector,
                    float       scale )
  {
    SDF_Vector  output;


    output.x = vector.x * scale;
    output.y = vector.y * scale;

    return output;
  }

  FT_LOCAL( float )
  sdf_vector_dot( SDF_Vector  a, 
                  SDF_Vector  b )
  {
    return ( ( a.x * b.x ) + ( a.y * b.y ) );
  }

  FT_LOCAL( float )
  sdf_vector_cross( SDF_Vector  a,
                    SDF_Vector  b )
  {
    return ( ( a.x * b.y ) - ( a.y * b.x ) );
  }

  FT_LOCAL_DEF( FT_Error )
  get_min_distance( SDF_Contour*       contour,
                    const SDF_Vector   point,
                    float             *min_distance )
  {
    /* compute shortest distance from `point' to the `contour' */

    FT_Error  error = FT_Err_Ok;


    if ( !contour || !min_distance )
      return FT_THROW( Invalid_Argument );

    switch ( contour->contour_type ) {
    case SDF_CONTOUR_TYPE_LINE:
    {
      /* in order to calculate the mimimum distance from a point to  */
      /* a line segment.                                             */
      /*                                                             */
      /* a = start point of the line segment                         */
      /* b = end point of the line segment                           */
      /* p = point from which mimimum distance is to be calculated   */
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

      const SDF_Vector  a                       = contour->start_pos;
      const SDF_Vector  b                       = contour->end_pos;
      const SDF_Vector  p                       = point;
      const SDF_Vector  line_segment            = sdf_vector_sub( b, a );
      const SDF_Vector  p_sub_a                 = sdf_vector_sub( p, a );

      SDF_Vector        nearest_point           = zero_vector;
      SDF_Vector        final_dist              = zero_vector;

      float             segment_squared_length  = 0.0f;
      float             factor                  = 0.0f;
      float             sign                    = 0.0f;


      factor = sdf_vector_dot( p_sub_a, line_segment ) /
               sdf_vector_squared_length( line_segment );
      factor = clamp( factor, 0.0f, 1.0f );

      nearest_point = sdf_vector_scale( line_segment, factor );
      nearest_point = sdf_vector_add( a, nearest_point );

      final_dist = sdf_vector_sub( nearest_point, p );

      /* now that we have the shortest  distance vector from `p' to the */
      /* line, we can determine the side at which the `p' lies with     */
      /* respect to line by simply taking the cross product of vectors  */
      /* `final_dist' and `line_segment'.                               */

      sign = sdf_vector_cross( final_dist, line_segment );
      if (sign < 0.0f)
      {
        /* point inside the outline */
        *min_distance = sdf_vector_length( final_dist );
      }
      else
      {
        /* point outside the outline */
        *min_distance = -sdf_vector_length( final_dist );
      }

      break;
    }
    case SDF_CONTOUR_TYPE_QUADRATIC_BEZIER:
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
      /* p = point from which mimimum distance is to be calculated   */
      /* ----------------------------------------------------------- */
      /* => the equation of a quadratic bezier curve can be written  */
      /*    B( t ) = ( 1 - t^2 )p0 + 2( 1 - t )tp1 + t^2p2           */
      /*    here t is the factor with range [0.0f, 1.0f]             */
      /*    the above equation can be rewritten as                   */
      /*    B( t ) = t^2( p0 - 2p1 + p2 ) + 2t( p1 - p0 ) + p0       */
      /*                                                             */
      /*    now put A = ( p0 - 2p1 + p2), B = ( p1 - p0 )            */
      /*    B( t ) = t^2( A ) + 2t( B ) + p0                         */
      /*                                                             */
      /* => the derivative of the above equation is written as       */
      /*    B`( t ) = 2( tA + B )                                    */
      /*                                                             */
      /* => now to find the shortest distance from p to B( t ), we   */
      /*    make find the point on the curve at which the shortest   */
      /*    distance vector ( i.e. B( t ) - p ) and the direction    */
      /*    ( i.e. B`( t )) makes 90 degrees. in other words we make */
      /*    the dot product zero.                                    */
      /*    ( B( t ) - p ).( B`( t ) ) = 0                           */
      /*    ( t^2( A ) + 2t( B ) + p0 - p ).( 2( tA + B ) ) = 0      */
      /*                                                             */
      /*    after simplifying we get a cubic equation as             */
      /*    at^3 + bt^2 + ct + d = 0                                 */
      /*    a = ( A.A ), b = ( 3A.B ), c = ( B.B + A.p0 - A.p )      */
      /*    d = ( p0.B - p.B )                                       */
      /*                                                             */
      /* => now the roots of the equation can be computed using the  */
      /*    `Cubic Formula'                                          */
      /*    ( https://mathworld.wolfram.com/CubicFormula.html )      */
      /*    we distacard the roots which do not lie in the range     */
      /*    [0.0f, 1.0f] and also check the endpoints ( p0, p2 )     */

    }
    case SDF_CONTOUR_TYPE_CUBIC_BEZIER:
    default:
      error = FT_THROW( Invalid_Argument );
    }

    return error;
  }

/* END */
