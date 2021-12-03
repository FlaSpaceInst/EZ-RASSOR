using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class changeTerrain : MonoBehaviour
{
    public Terrain TerrainMain;

    /*void OnGUI()
    {
        if(GUI.Button (new Rect(30,30,200,30), "Change terrain height bruv"))
        {
            //get heightmap width and height 
            int xRes = TerrainMain.terrainData.heightmapResolution;
            int yRes = TerrainMain.terrainData.heightmapResolution;

            

            //GetHeights- basically gets the heightmap points on the terrainds
            //Get height of the position that the rover is in, will need to scale the heightmap transformation to be the size of a standard dig.

            float [,] heights = TerrainMain.terrainData.GetHeights (0,0, xRes, yRes);

            //manipulate the terrain ( heightmap data ), 0 - 1., 1 being the maximum possible height.
            //heights[10, 10] = 1f; 
            //heights[20, 20] = 0.5f; 

            Debug.Log (heights[50,50]);  
            heights[50, 50] = 1.1f * heights[50,50];
             

            
            //SetHeights to set it lol
            TerrainMain.terrainData.SetHeights (0, 0, heights);
            Debug.Log (heights[50,50]);
        }
    }*/
}
