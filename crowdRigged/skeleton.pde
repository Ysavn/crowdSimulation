int tubeRes = 32;
float[] tubeX = new float[tubeRes+1];
float[] tubeY = new float[tubeRes+1];
PImage img;
float dRadiusFact = 2.5;

public class RigNode{
  public Vec2 cur;
  public Vec2 dest;
  public float r;
  public Vec3 col; //rgb
  public RigNode(){
    cur = new Vec2(0, 0);
    //dest = new Vec2(width/2, height/2-100);
    dest = new Vec2(150, 50);
  }
}

//Returns true if the point is inside a circle
boolean pointInCircle(Vec2 center, float r, Vec2 pointPos){
  float dist = pointPos.distanceTo(center);
  if (dist < r+2){ //small safty factor
    return true;
  }
  return false;
}

//Returns true if the point is inside a list of circle
boolean pointInNodeList(Vec2 pointPos, float rad){
  
  //destination check
  for (int i = 0; i < totalNodes; i++){
    Vec2 center =  skeleton[i].dest;
    float r = skeleton[i].r/dRadiusFact + rad;
    if (pointInCircle(center,r,pointPos)){
      return true;
    }
  }
  
  //existing placed nodes check
  for (int i = 0; i < totalNodes; i++){
    Vec2 center =  skeleton[i].cur;
    float r = skeleton[i].r + rad;
    if (pointInCircle(center,r,pointPos)){
      return true;
    }
  }
  return false;
}

void placeRandomNodes(){
  for(int i=0;i<totalNodes;i++){
    boolean flag = false;
    Vec2 p;
    do{
      p = new Vec2(random(20, width-20), random(20, height-20));
      flag = pointInNodeList(p, skeleton[i].r);
      
    }while(flag);
    skeleton[i].cur = p;
  }
}

RigNode[] skeleton;
