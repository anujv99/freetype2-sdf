
#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_ERRORS_H

#include "sdfgen.h"

  FT_EXPORT_DEF(FT_Error)
  FT_Generate_SDF( FT_Library   library,
                   FT_Outline*  outline, 
                   FT_Bitmap   *abitmap )
  {
    return FT_Err_Unimplemented_Feature;
  }
