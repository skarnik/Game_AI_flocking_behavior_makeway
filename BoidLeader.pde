
class BoidLeader extends Boid
{
  //boid leader
  BoidLeader(){
  
    position = new PVector(width/2, height/2);
    velocity = new PVector(0,0);
    destination = new PVector(0,0);
   
    direction = new PVector(0,0);

    max_speed = 110; //100
    max_accel = 40; //100
  
    roS = 3;
    roD = 500;
    r_rotS = radians(1);
    r_rotD = 5;
    max_ang_accel = 40;
    max_rotation  = 0.9;
  
    time_to_target = 1.3;
    time_to_target_rot = 1.2;
    opacity = 255;
    scale = 1;
    boidcolor = color(255, 255, 255);
    crumbcolor = color(255, 255, 255);
    
  }
  
  void run(){
   findMouse();
    moveRandom();
   
    seek();
    breadcrumb_trail();
  
  }
  
}


