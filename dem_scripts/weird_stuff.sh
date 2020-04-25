rm -rf another/
rm -rf ~/.gazebo/models/another/
rm -rf ~/.gazebo/paging/APOLLO17_ORTHOMOSAIC_50CM_tile_0_5289_converted/
mkdir another/
cp -a "./s_pole/." another/
cp ~/Desktop/APOLLO17_ORTHOMOSAIC_50CM_tile_0_5289_converted.jpg another/materials/textures/APOLLO17_ORTHOMOSAIC_50CM_tile_0_5289_converted.jpg 
rm another/materials/textures/LRO_PNG_TEST_257.jpg
python replace.py "./another/moon_model.sdf" another APOLLO17_ORTHOMOSAIC_50CM_tile_0_5289_converted.jpg
cp -r another/ ~/.gazebo/models/
