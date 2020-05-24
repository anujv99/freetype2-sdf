
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

  FT_LOCAL_DEF( FT_UShort )
  solve_quadratic_equation( float  a,
                            float  b,
                            float  c,
                            float  out[2] )
  {
    float  discriminant       = 0.0f;
    float  discriminant_root  = 0.0f;


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
  solve_cubic_equation( float a,
                        float b,
                        float c,
                        float d,
                        float out[3] )
  {
    /* the method here is a direct implementation of the  */
    /* `Cardano's Cubic formula' given here               */
    /* https://mathworld.wolfram.com/CubicFormula.html    */

    float  q              = 0.0f;  /* intermediate      */
    float  r              = 0.0f;  /* intermediate      */
    float  discriminant   = 0.0f;  /* discriminant      */

    float  a2             = b;     /* x^2 coefficients  */
    float  a1             = c;     /* x coefficients    */
    float  a0             = d;     /* constant          */

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
      float  t = 0.0f;  /* angle theta */


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
      float  s = 0.0f;  /* intermediate */


      /* all real roots and at least two are equal */
      s = cbrtf( r );
      a2 /= -3.0f;
      out[0] = a2 + ( 2 * s );
      out[1] = a2 - s;

      return 2;
    }
    else /* discriminant > 0.0f */
    {
      float  s = 0.0f;  /* intermediate */
      float  t = 0.0f;  /* intermediate */


      /* only one real root */
      discriminant = sqrtf( discriminant );
      s = cbrtf( r + discriminant );
      t = cbrtf( r - discriminant );
      a2 /= -3.0f;
      out[0] = a2 + ( s + t );

      return 1;
    }

  }

  FT_LOCAL_DEF( FT_UShort )
  solve_quartic_equation( float  a,
                          float  b,
                          float  c,
                          float  d,
                          float  e,
                          float  out[4] )
  {
    /* the method here is a direct implementation of the  */
    /* https://mathworld.wolfram.com/QuarticEquation.html */


    /* coefficients of the quartic equation */
    float  a3 = b;
    float  a2 = c;
    float  a1 = d;
    float  a0 = e;

    /* coefficients of the resolvent cubic */
    float  y2          = 0.0f;
    float  y1          = 0.0f;
    float  y0          = 0.0f;
    float  y_out[3]    = { 0.0f, 0.0f, 0.0f };

    /* the roots from y_out that will be chosen for computation */
    float  y_chosen    = 0.0f;

    FT_UShort y_roots  = 0;

    /* coefficients of the final quadratic */
    float x1           = 0.0f;
    float x0           = 0.0f;

    /* total number of roots of quartic */
    FT_UShort t_roots  = 0;


    if ( a == 0.0f )
    {
      return solve_cubic_equation( b, c, d, e, out );
    }
    if ( a != 1.0f )
    {
      /* Normalize the coefficients */
      a3 /= a;
      a2 /= a;
      a1 /= a;
      a0 /= a;
    }

    y2 = -a2;
    y1 = ( a1 * a3 ) - ( 4 * a0 );
    y0 = ( 4 * a2 * a0 ) - ( a1 * a1 ) - ( a3 * a3 * a0 );
    y_roots = solve_cubic_equation( 1.0f, y2, y1, y0, y_out );
    y_chosen = y_out[0];

    /* choose the root with max absolute value in case of multiple roots */
    if (y_roots == 2)
    {
      if ( fabs( y_out[1] ) > fabs( y_chosen ) ) y_chosen = y_out[1];
    }
    else if ( y_roots == 3 )
    {
      if ( fabs( y_out[1] ) > fabs( y_chosen ) ) y_chosen = y_out[1];
      if ( fabs( y_out[2] ) > fabs( y_chosen ) ) y_chosen = y_out[2];
    }


    /* now use the chosen root of the resolvent cubic to find the roots */

    x1 = ( a3 * a3 ) - ( 4 * a2 ) + ( 4 * y_chosen );
    x0 = ( y_chosen * y_chosen ) - ( 4 * a0 );

    x1 = sqrtf( x1 );
    x0 = sqrtf( x0 );

    t_roots  = solve_quadratic_equation( 1.0f, ( a3 + x1 ) / 2.0f, 
                                         ( y_chosen - x0 ) / 2.0f, 
                                         out );
    t_roots += solve_quadratic_equation( 1.0f, ( a3 - x1 ) / 2.0f, 
                                         ( y_chosen + x0 ) / 2.0f, 
                                         out + t_roots );

    return t_roots;
  }

  static int
  qsort_compare_float( const void * a,
                       const void * b )
  {
    return ( *(float*)a > *(float*)b );
  }

  FT_LOCAL( FT_UShort )
  solve_quintic_equation( float  a,
                          float  b,
                          float  c,
                          float  d,
                          float  e,
                          float  f,
                          float  min,
                          float  max,
                          float  out[5] )
  {
    /* output roots of the quintic equation */
    float  out_roots[5]  = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    /* bounds calculated by the isolator polynomial */
    float  bounds[6]     = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    FT_UShort  t_roots   = 0; /* total number of roots  */
    FT_UShort  t_bounds  = 0; /* total number of bounds */

    /* coefficients of the original equation */
    float      a4  = b;
    float      a3  = c;
    float      a2  = d;
    float      a1  = e;
    float      a0  = f;

    /* coefficients of the depressed form of equation */
    float      d3  = 0.0f;
    float      d2  = 0.0f;
    float      d1  = 0.0f;
    float      d0  = 0.0f;

    if ( min > max )
    {
      float  temp = 0.0f; 


      /* max min < max */
      temp = min;
      min = max;
      max = temp;
    }

    if ( a == 0.0f )
    {
      FT_UShort  temp  = 0;
      FT_Int     i     = 0;

      /* quartic equation */
      t_roots = solve_quartic_equation( b, c, d, e, f, out_roots );
      temp = 0;
      for ( i = 0; i < t_roots; i++ )
      {
        if (out_roots[i] >= min && out_roots[i] <= max)
          out[temp++] = out_roots[i];
      }

      return temp;
    }

    if ( a != 1.0f )
    {
      /* Normalize the coefficients */
      a4 /= a;
      a3 /= a;
      a2 /= a;
      a1 /= a;
      a0 /= a;
    }

    /* convert the equation to `depressed' form         */
    /* i.e. x^5 + ( d3 )x^3 + ( d2 )x2^2 + ( d3 )x + d4 */
    /* we eleminate the x^( t - 1 ) term to make the eq */
    /* in depressed form. where t is the degree of eq.  */
    /* to convert a 5th degree equiation to depressed   */
    /* form simply put f( x ) -> f( x - a4 / 5 )        */

    if ( a4 == 0.0f )
    {
      /* already in depressed form */
      d3 = a3;
      d2 = a2;
      d1 = a1;
      d0 = a0;
    }
    else
    {
      float  a42 = a4  * a4; /* ( a4 )^2  */
      float  a43 = a42 * a4; /* ( a4 )^3  */
      float  a44 = a43 * a4; /* ( a4 )^4  */
      float  a45 = a44 * a4; /* ( a4 )^5  */

      d3 = a3 - ( 2.0f * a42 ) / 5.0f;
      d2 = ( 4.0f * a43 ) / 25.0f - ( 3.0f * a4 * a3 )  / 5.0f + a2;
      d1 = ( -3.0f * a44 ) / 125.0f +
           ( 3.0f * a42 * a3 ) / 25.0f  -
           ( 2.0f * a4 * a2 ) / 5.0f   + a1;
      d0 = ( 4.0f * a45 ) / 3125.0f - ( a43 * a3 ) / 125.0f +
           ( a42 * a2 ) / 25.0f - ( a4 * a1 ) / 5.0f + a0;
    }

    {
      /* calculate isolator polynomials to find the */
      /* range where the roots lie.                 */
      /* http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.133.2233&rep=rep1&type=pdf */

      float      ia2        = 0.0f;
      float      ia1        = 0.0f;
      float      ia0        = 0.0f;
                            
      float      ib2        = 0.0f;
      float      ib1        = 0.0f;
      float      ib0        = 0.0f;
                 
      float      roots[4]   = { 0.0f, 0.0f, 0.0f, 0.0f };
      FT_UShort  num_roots  = 0;
      FT_UShort  i          = 0;

      float      d32        = d3 * d3;


      ia2 = ( 12.0f * d32 * d3 ) + ( 45.0f * d2 * d2 ) - ( 40.0f * d3 * d1 );
      ia1 = ( 8.0f * d32 * d2 ) + ( 60.0f * d2 * d1 ) - ( 50.0f * d3 * d0 );
      ia0 = ( 4.0f * d32 * d1 ) + ( 75.0f * d2 * d0);

      ib2 = 10.0f * d3;
      ib1 = -15.0f * d2;
      ib0 = 4.0f * d32;

      num_roots  = solve_quadratic_equation( ia2, ia1, ia0, roots );
      num_roots += solve_quadratic_equation( ib2, ib1, ib0, roots + num_roots );

      if ( a4 != 0.0f )
      {
        /* we convert the equation to depressed form `a4' != 0 */
        /* so the bounds of the original equation will be:     */
        for ( i = 0; i < num_roots; i++ )
        {
          roots[i] -= ( a4 / 5.0f );
        }
      }

      /* sort the bounds */
      qsort( ( void * )roots, ( rsize_t )num_roots, sizeof( roots[i] ), 
               qsort_compare_float );

      /* we will only check for roots within range [min, max] */
      bounds[0] = min;
      t_bounds = 1;
      for ( i = 0; i < num_roots; i++ )
      {
        if ( roots[i] > min && roots[i] < max )
        {
          bounds[t_bounds] = roots[i];
          t_bounds++;
        }
      }
      bounds[t_bounds] = max;
      t_bounds++;
    }

    return 0;
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

  FT_LOCAL_DEF( float )
  sdf_vector_dot( SDF_Vector  a, 
                  SDF_Vector  b )
  {
    return ( ( a.x * b.x ) + ( a.y * b.y ) );
  }

  FT_LOCAL_DEF( float )
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
  get_min_distance( SDF_Contour*       contour,
                    const SDF_Vector   point,
                    SDF_Vector        *shortest_point,
                    SDF_Vector        *curve_dir )
  {
    /* compute shortest distance from `point' to the `contour' */

    FT_Error  error = FT_Err_Ok;


    if ( !contour || !shortest_point )
      return FT_THROW( Invalid_Argument );

    switch ( contour->contour_type ) {
    case SDF_CONTOUR_TYPE_LINE:
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

      const SDF_Vector  a                       = contour->start_pos;
      const SDF_Vector  b                       = contour->end_pos;
      const SDF_Vector  p                       = point;
      const SDF_Vector  line_segment            = sdf_vector_sub( b, a );
      const SDF_Vector  p_sub_a                 = sdf_vector_sub( p, a );

      SDF_Vector        nearest_point           = zero_vector;

      float             segment_squared_length  = 0.0f;
      float             factor                  = 0.0f;
      float             sign                    = 0.0f;


      /* if both the endpoints of the line segmnet coincide then the   */
      /* shortest distance will be from `point' to any of the endpoint */
      if ( sdf_vector_equal( a, b ) == 1 )
      {
        *shortest_point = a;
        break;
      }

      factor = sdf_vector_dot( p_sub_a, line_segment ) /
               sdf_vector_squared_length( line_segment );
      factor = clamp( factor, 0.0f, 1.0f );

      nearest_point = sdf_vector_scale( line_segment, factor );
      nearest_point = sdf_vector_add( a, nearest_point );

      *shortest_point = nearest_point;
      *curve_dir = sdf_vector_normalize( line_segment );

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

      SDF_Vector  aA             = zero_vector; /* A in the above comment */
      SDF_Vector  bB             = zero_vector; /* B in the above comment */
      SDF_Vector  nearest_point  = zero_vector;

      SDF_Vector  p0             = contour->start_pos;
      SDF_Vector  p1             = contour->control_point_a;
      SDF_Vector  p2             = contour->end_pos;
      SDF_Vector  p              = point;

      /* cubic coefficients */
      float       a        = 0.0f;
      float       b        = 0.0f;
      float       c        = 0.0f;
      float       d        = 0.0f;
      float       min      = FLT_MAX; /* shortest distace */

      float       roots[5] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

      FT_UShort   num_r    = 0;   /* number of roots */
      FT_UShort   i        = 0;


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
        float  t                    = roots[i];
        float  t2                   = t * t;
        float  dist                 = 0.0f;
        
        SDF_Vector curve_point      = zero_vector;  /* point on the curve */
        SDF_Vector temp             = zero_vector;


        /* only check of t in range [0.0f, 1.0f] */
        if ( t < 0.0f || t > 1.0f )
          continue;

        /* B( t ) = t^2( A ) + 2t( B ) + p0  */
        curve_point  = sdf_vector_scale( aA, t2 );
        temp = sdf_vector_scale( bB, 2 * t );
        curve_point = sdf_vector_add( curve_point, temp );
        curve_point = sdf_vector_add( curve_point, p0 );

        /* compute distance from `point' to the `curve_point' */
        temp = sdf_vector_sub( curve_point, p );
        dist = sdf_vector_length( temp );

        if ( dist < min )
        {
          min = dist;
          nearest_point = curve_point;
        }
      }

      *shortest_point = nearest_point;
      break;
    }
    case SDF_CONTOUR_TYPE_CUBIC_BEZIER:
    {
      /* the procedure to find the shortest distance from a point to */
      /* a cubci bezier curve is simliar to a quadratic curve.       */
      /* The only difference is that while calculating the factor    */
      /* `t', instead of a cubic polynomial equation we have to find */
      /* the roots of a 5th degree polynomial equation.              */
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
      /*            B = ( p0 - 2p1 + p2 ), C = ( -p0 + p1 )          */
      /*    B( t ) = t^3( A ) + 3t^2( B ) + 3tC + p0                 */
      /*                                                             */
      /* => the derivative of the above equation is written as       */
      /*    B`( t ) = 3t^2( A ) + 6t( B ) + 3C                       */
      /*                                                             */
      /* => now similar to quadratic bezier, to find the nearest     */
      /*    point on the curve from `p', we make the dot product     */
      /*    zero i.e. ( B( t ) - p ).( B`( t ) ) = 0, we get:        */
      /*    ( t^3( A ) + 3t^2( B ) + 3tC + p0 - p ).                 */
      /*    ( 3t^2( A ) + 6t( B ) + 3C) = 0                          */
      /*                                                             */
      /*    in the above equation put D = ( p0 - p )                 */
      /*                                                             */
      /* => finally we get a 5th order polynomial equation:          */
      /*    t^5( A.A ) + t^4( 5A.C ) + t^3( 4A.C + 6B.B ) +          */
      /*    t^2( 9C.B + A.D ) + t( 3C.C + 2B.D ) + C.D               */
      /*                                                             */
      /* => the roots within range [0.0f, 1.0f] will give us the     */
      /*    factor `t' at which the curve B( t ) makes 90 degree     */
      /*    with ( B( t ) - p ), and hence that will be the shortest */
      /*    distance. But we also have to check the endpoints `p0'   */
      /*    and `p1' because they can be even closer to the point    */
      /*    `p'.                                                     */
      /*                                                             */
      /* [note]: B and B( t ) are different in the above equations   */

      
    }
    default:
      error = FT_THROW( Invalid_Argument );
    }

    return error;
  }

/* END */
