PImage img;
int totalNodes;
float k_goal = 10;
float k_avoid = 500;
float tH = 0.75;
float agentRad = 20;
float agentRadDraw = 30;
float dt = 0.009;

//The agent states
Vec2[] agentPos;
Vec2[] agentVel;
Vec2[] agentAcc;
Vec3[] agentCol;
float[] goalSpeed;
int totalReached = 0;

//The agent goals
Vec2[] goalPos;

void setup(){
  img = loadImage("poolTable.jpg");
  totalNodes = 6;
  size(512,307);
  agentPos = new Vec2[totalNodes];
  agentVel = new Vec2[totalNodes];
  agentAcc = new Vec2[totalNodes];
  goalPos = new Vec2[totalNodes];
  goalSpeed = new float[totalNodes];
  agentCol = new Vec3[totalNodes];
  
  
  //Set initial agent positions and goals
  agentPos[0] = new Vec2(50,50); 
  agentPos[1] = new Vec2(256,50);
  agentPos[2] = new Vec2(462,50);
  agentPos[3] = new Vec2(462, 257);
  agentPos[4] = new Vec2(256, 257);
  agentPos[5] = new Vec2(50, 257);
  
  goalPos[0] = new Vec2(462, 257);
  goalPos[1] = new Vec2(256, 257);
  goalPos[2] = new Vec2(50, 257);
  goalPos[3] = new Vec2(50,50); 
  goalPos[4] = new Vec2(256,50);
  goalPos[5] = new Vec2(462,50);
  
  agentCol[0] = new Vec3(255, 0, 0);
  agentCol[1] = new Vec3(0, 0, 255);
  agentCol[2] = new Vec3(255, 255, 255);
  agentCol[3] = new Vec3(255, 255, 0);
  agentCol[4] = new Vec3(128, 0, 128);
  agentCol[5] = new Vec3(150, 75, 0);
  
  for(int i=0;i<totalNodes;i++){
    goalSpeed[i] = 50;
  }
  
  float dist1 = agentPos[0].distanceTo(goalPos[0]);
  float dist2 = agentPos[1].distanceTo(goalPos[1]);
  goalSpeed[1] = (dist2/dist1)*goalSpeed[1];
  goalSpeed[4] = (dist2/dist1)*goalSpeed[4];
 
  //Set initial velocities to cary agents towards their goals
  for (int i = 0; i < totalNodes; i++){
    agentVel[i] = goalPos[i].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed[i]);
  }
}

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
  
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
  
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.lengthSqr(); 
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
  
  float d = b*b - 4*a*c; //discriminant 
  
  if (d >=0 ){ 
    //If d is positive we know the line is colliding
    float t = (-b - sqrt(d))/(2*a); //Optimization: we typically only need the first collision! 
    if (t >= 0) return t;
    return -1;
  }
  
  return -1; //We are not colliding, so there is no good t to return 
}

//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2, float tH){
  float r = radius1 + radius2;
  Vec2 dir = vel1.minus(vel2);
  
  float t = rayCircleIntersectTime(pos2, r, pos1, dir);
  return t;
}

// Compute attractive forces to draw agents to their goals, 
// and avoidance forces to anticipatory avoid collisions
Vec2 computeAgentForces(int id){
  
  Vec2 goalVel = goalPos[id].minus(agentPos[id]);
  
  Vec2 Fg = goalVel.minus(agentVel[id]).times(k_goal);
  Vec2 Fa = new Vec2(0, 0);
  
  if(goalVel.length() < 2){
    agentVel[id] = new Vec2(0, 0);
    agentPos[id] = goalPos[id];
    return Fa;
  }
  
  for(int i=0;i<totalNodes;i++){
    if(i==id) continue;
    float t = computeTTC(agentPos[id], agentVel[id], agentRad, agentPos[i], agentVel[i], agentRad, tH);
    //if(t==0) paused = true;
    if(t==-1 || t>tH) continue;
    Vec2 dir1 = agentPos[id].plus(agentVel[id].times(t));
    Vec2 dir2 = agentPos[i].plus(agentVel[i].times(t));
    Vec2 dir = dir1.minus(dir2).normalized();
    float mag = k_avoid/(t + 0.01);
    Fa.add(dir.times(mag));
  }
  
  return Fa.plus(Fg);

}


//Update agent positions & velocities based acceleration
void moveAgent(float dt){
  //Compute accelerations for every agents
  for (int i = 0; i < totalNodes; i++){
    agentAcc[i] = computeAgentForces(i);
  }
  //Update position and velocity using (Eulerian) numerical integration
  for (int i = 0; i < totalNodes; i++){
    agentVel[i].add(agentAcc[i].times(dt));
    agentPos[i].add(agentVel[i].times(dt));
  }
}

boolean paused = true;
void draw(){
  background(img);
  
  //Update agent if not paused
  if (!paused){
    moveAgent(dt);
  }
  
  for(int i=0;i<totalNodes;i++){
    fill(agentCol[i].x, agentCol[i].y, agentCol[i].z);
    circle(agentPos[i].x, agentPos[i].y, agentRad);
  }
  
 
  
}

//Pause/unpause the simulation
void keyPressed(){
  if (key == ' ') paused = !paused;
}

class hitInfo{
  public boolean hit = false;
  public float t = -1;
}

///////////////////////

//Special versions of rayCircleIntersect() that don't assume a normalized length
//Q: Why don't we assume the length is normalized?

float rayCircleIntersect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float max_t){
  
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
  
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length();  
  float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
  float c = toCircle.lengthSqr() - (r*r); //different of squared distances
  
  float d = b*b - 4*a*c; //discriminant 
  
  if (d >=0 ){ 
    //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
    //  ... this means t will be between 0 and the length of the line segment
    float t1 = (-b - sqrt(d))/(2*a); 
    float t2 = (-b + sqrt(d))/(2*a); 
    //println(hit.t,t1,t2);
    if (t1 > 0 && t1 < max_t){ //We intersect the circle
      return t1;
    }
    else if (t1 < 0 && t2 > 0){ //We start in the circle
      //println(center.toString(), l_start.toString(), max_t);
      return 0;
    }
  }
  return -1;
}
