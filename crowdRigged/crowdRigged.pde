int totalNodes;
float k_goal = 2.3;
float k_avoid = 1.33;
float agentRad = 40;
float agentRadDraw = 30;
float goalSpeed = 100;
float tH = 3;
float dt = 0.008;

//The agent states
Vec2[] agentPos;
Vec2[] agentVel;
Vec2[] agentAcc;
boolean[] reachedGoal;
int totalReached = 0;

//The agent goals
Vec2[] goalPos;

//skeleton graph
ArrayList<Integer>[] neighbors; 

void setup(){
  totalNodes = 15;
  size(600,600);
  agentPos = new Vec2[totalNodes];
  agentVel = new Vec2[totalNodes];
  agentAcc = new Vec2[totalNodes];
  goalPos = new Vec2[totalNodes];
  reachedGoal = new boolean[totalNodes];
  neighbors = new ArrayList[totalNodes]; 
  
  skeleton = new RigNode[totalNodes];
  for(int i=0;i<totalNodes;i++) skeleton[i] = new RigNode();
  
  //head
  skeleton[0].dest.add(new Vec2(0, -5));
  skeleton[0].r = 25;
  skeleton[0].col = new Vec3(255, 255, 0);
  
  //neck
  skeleton[1].dest.add(new Vec2(0, 30));
  skeleton[1].r = 20;
  skeleton[1].col = new Vec3(128, 255, 0);
  
  //shoulder right
  skeleton[2].dest.add(new Vec2(30, 20));
  skeleton[2].r = 20;
  skeleton[2].col = new Vec3(255, 128, 0);
  
  //elbow right
  skeleton[3].dest.add(new Vec2(50, 40));
  skeleton[3].r = 15;
  skeleton[3].col = new Vec3(160, 160, 160);
  
  //wrist right
  skeleton[4].dest.add(new Vec2(50, 85));
  skeleton[4].r = 18;
  skeleton[4].col = new Vec3(255, 0, 0);
  
  //shoulder left
  skeleton[5].dest.add(new Vec2(-30, 20));
  skeleton[5].r = 20;
  skeleton[5].col = new Vec3(255, 128, 0);
  
  //elbow left
  skeleton[6].dest.add(new Vec2(-50, 40));
  skeleton[6].r = 15;
  skeleton[6].col = new Vec3(160, 160, 160);
  
  //wrist left
  skeleton[7].dest.add(new Vec2(-50, 85));
  skeleton[7].r = 18;
  skeleton[7].col = new Vec3(255, 0, 0);
  
  //abs
  skeleton[8].dest.add(new Vec2(0, 70));
  skeleton[8].r = 20;
  skeleton[8].col = new Vec3(128, 255, 0);
  
  //groin left
  skeleton[9].dest.add(new Vec2(-20, 90));
  skeleton[9].r = 18;
  skeleton[9].col = new Vec3(51, 153, 255);
  
  //knee left
  skeleton[10].dest.add(new Vec2(-30, 140));
  skeleton[10].r = 15;
  skeleton[10].col = new Vec3(204, 102, 0);
  
  //ankle left
  skeleton[11].dest.add(new Vec2(-30, 200));
  skeleton[11].r = 20;
  skeleton[11].col = new Vec3(102, 51, 0);
  
  //groin right
  skeleton[12].dest.add(new Vec2(20, 90));
  skeleton[12].r = 18;
  skeleton[12].col = new Vec3(51, 153, 255);
  
  //knee right
  skeleton[13].dest.add(new Vec2(30, 140));
  skeleton[13].r = 15;
  skeleton[13].col = new Vec3(204, 102, 0);
  
  //ankle right
  skeleton[14].dest.add(new Vec2(30, 200));
  skeleton[14].r = 20;
  skeleton[14].col = new Vec3(102, 51, 0);
  
  for(int i=0;i<totalNodes;i++){
    skeleton[i].dest.mul(2);
  }
  
  placeRandomNodes();
  
  for(int i=0;i<totalNodes;i++){
    agentPos[i] = skeleton[i].cur;
    goalPos[i] = skeleton[i].dest;
    reachedGoal[i] = false;
    neighbors[i] = new ArrayList<Integer>();
  }
  
  //Building the skeleton graph  
  neighbors[0].add(1);
  neighbors[1].add(2);
  neighbors[1].add(5);
  neighbors[1].add(8);
  neighbors[2].add(3);
  neighbors[3].add(4);
  neighbors[5].add(6);
  neighbors[6].add(7);
  neighbors[8].add(9);
  neighbors[8].add(12);
  neighbors[9].add(10);
  neighbors[10].add(11);
  neighbors[12].add(13);
  neighbors[13].add(14);
  
  //Set initial velocities to cary agents towards their goals
  for (int i = 0; i < totalNodes; i++){
    agentVel[i] = goalPos[i].minus(agentPos[i]);
    if (agentVel[i].length() > 0)
      agentVel[i].setToLength(goalSpeed);
  }
}

//Return at what time agents 1 and 2 collide if they keep their current velocities
// or -1 if there is no collision.
float computeTTC(Vec2 pos1, Vec2 vel1, float radius1, Vec2 pos2, Vec2 vel2, float radius2){
  float r = radius1 + radius2;
  Vec2 dir = vel1.minus(vel2);
  
  return rayCircleIntersect(pos2, r, pos1, dir, tH);
}

// Compute attractive forces to draw agents to their goals, 
// and avoidance forces to anticipatory avoid collisions
Vec2 computeAgentForces(int id){
  
  Vec2 goalVel = goalPos[id].minus(agentPos[id]);
  
  Vec2 Fg = goalVel.minus(agentVel[id]).times(k_goal);
  Vec2 Fa = new Vec2(0, 0);
  
  if(goalVel.length() < 3){
    agentVel[id] = new Vec2(0, 0);
    agentPos[id] = goalPos[id];
    if(!reachedGoal[id]) totalReached++;
    reachedGoal[id] = true;
    return Fa;
  }
  
  for(int i=0;i<totalNodes;i++){
    if(i==id) continue;
    float t = computeTTC(agentPos[id], agentVel[id], skeleton[id].r, agentPos[i], agentVel[i], skeleton[i].r);
    if(t==-1) continue;
    Vec2 dir = agentPos[id].plus(agentVel[id].times(t)).minus(agentPos[i].plus(agentVel[i].times(t))).normalized();
    float mag = k_avoid/(t + 0.001);
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
  background(0);
  
  //Update agent if not paused
  if (!paused){
    moveAgent(dt);
  }

  //dest loc
  for(int i=0;i<totalNodes;i++){
    fill(skeleton[i].col.x, skeleton[i].col.y, skeleton[i].col.z);
    circle(skeleton[i].dest.x, skeleton[i].dest.y, skeleton[i].r/dRadiusFact);
  }
  
  //draw edges once all nodes reach destination
  if(totalReached==totalNodes){
    stroke(255);
    strokeWeight(5);
    for(int i=0;i<totalNodes;i++){
      Vec2 u = skeleton[i].dest;
      for(int j=0;j<neighbors[i].size();j++){
        Vec2 v = skeleton[neighbors[i].get(j)].dest;
        line(u.x, u.y, v.x, v.y);
      }
    }
  }
  
  stroke(0);
  strokeWeight(1);
  //curr loc
  for(int i=0;i<totalNodes;i++){
    fill(skeleton[i].col.x, skeleton[i].col.y, skeleton[i].col.z);
    circle(agentPos[i].x, agentPos[i].y, skeleton[i].r);
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
      return 0;
    }
  }
  return -1;
}

float rayCircleIntersectTime(Vec2 center, float r, Vec2 l_start, Vec2 l_dir){
  
  //Compute displacement vector pointing from the start of the line segment to the center of the circle
  Vec2 toCircle = center.minus(l_start);
  
  //Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
  float a = l_dir.length(); 
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
