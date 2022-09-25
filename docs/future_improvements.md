# Future improvements
This is mainly a collection of misc. TODOs from lidar_processing.cpp. If you're brave and daring,
feel free to give these a crack.

- Inpainting
  - Instead of doing inpaint, maybe try force opencv to do bicubic interpolation (faster). We don't
  actually really want to do inpainting in the first place because it's slow, and bicubic interpolation
  of missing pixels should suffice.
  - Ideally everything outside of the crop rectangle gets "filled in" more inaccurately
    probably using morphological operations (very large dilation)