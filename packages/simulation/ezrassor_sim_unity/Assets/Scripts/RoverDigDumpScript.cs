using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RoverDigDumpScript : MonoBehaviour
{
    
    // ezrassor game object references
    public GameObject ezrazzor1;
    public GameObject ezrazzor2;
    public GameObject ezrazzor3;
    public GameObject ezrazzor4;
    
    // references to terrain and its heightmap width and height
    public Terrain terr;
    int terrWidth;
    int terrHeight;
    
    // position of rassor
    int terrXPos;
    int terrYPos;
    
    // size will determine the area that will change under the rover when digging
    int size = 1;
    float newHeight;

    // refernce to create a copy of original heights of terrain map
    public TerrainData terrCopy;
    public float [,] OriginalHeights;
 
    void Awake () 
    {
        // save terrain heights original values 
        terr = Terrain.activeTerrain;
        terrWidth = terr.terrainData.heightmapWidth;
        terrHeight = terr.terrainData.heightmapHeight;
        OriginalHeights = terr.terrainData.GetHeights(0,0,terr.terrainData.heightmapWidth,terr.terrainData.heightmapHeight);
    }

    // used for setting back terrain to original values
    void OnApplicationQuit()
    {
        // set Heights of our terrain back to original heights before closing application
        terr.terrainData.SetHeights(0,0,OriginalHeights);
        Debug.Log("Exiting Application");
    }

    public void Dig(GameObject ezrassor)
    {
     // convert ezrassor world position into terrain coordinates
     Vector3 tempCoord = (ezrassor.transform.position - terr.gameObject.transform.position);
     Vector3 coord;
     coord.x = tempCoord.x / terr.terrainData.size.x;
     coord.y = tempCoord.y / terr.terrainData.size.y;
     coord.z = tempCoord.z / terr.terrainData.size.z;
 
     // grab ezrassor position
     terrXPos = (int) (coord.x * terrWidth); 
     terrYPos = (int) (coord.z * terrHeight);
 
     // offset to ensure digging is under the ezrassor gameobject
     int offset = size / 2;
 
     // grab terrain height 
     float[,] heights = terr.terrainData.GetHeights(terrXPos-offset,terrYPos-offset,size,size);
 
     newHeight =  heights[0,0] * 0.9999f;

     // uncomment if you need to increase are that will be dug under rover
     /*
     for (int i=0; i < size; i++)
         for (int j=0; j < size; j++)
     */

     heights[0,0] = newHeight;
 
     // raise terrain to new height over time
     newHeight += Time.deltaTime;
 
     // set terrain height
     terr.terrainData.SetHeightsDelayLOD(terrXPos-offset,terrYPos-offset,heights);
    }



    public void Dump(GameObject ezrassor)
    {
     // convert ezrassor world position into terrain coordinates
     Vector3 tempCoord = (ezrassor.transform.position - terr.gameObject.transform.position);
     Vector3 coord;
     coord.x = tempCoord.x / terr.terrainData.size.x;
     coord.y = tempCoord.y / terr.terrainData.size.y;
     coord.z = tempCoord.z / terr.terrainData.size.z;
 
     // grab ezrassor position
     terrXPos = (int) (coord.x * terrWidth); 
     terrYPos = (int) (coord.z * terrHeight); 
 
     // offset to ensure digging is under the ezrassor gameobject
     int offset = size / 2;
 
     // grab terrain height 
     float[,] heights = terr.terrainData.GetHeights(terrXPos-offset,terrYPos-offset,size,size);
 
     newHeight =  heights[0,0] * 1.0001f;

     // uncomment if you need to increase are that will be dug under rover
     /*
     for (int i=0; i < size; i++)
         for (int j=0; j < size; j++)
     */
     heights[0,0] = newHeight;
 
     // raise terrain to new height over time
     newHeight += Time.deltaTime;
 
     // set terrain height
     terr.terrainData.SetHeightsDelayLOD(terrXPos-offset,terrYPos-offset,heights);
    }
    

    

}
