

class FlockBoid extends Boid
{
  
  //flockboids
  color crumbcolor = color(255, 255, 255);
  color boidcolor = color(255, 255, 255);
  
  FlockBoid(){

  
    position = new PVector(width/2, height/2);
    velocity = new PVector(0,0);
    destination = new PVector(0,0);
    orientation = radians(0);
    direction = new PVector(0,0);

    max_speed = 60; //100
    max_accel = 27.0; //100
  
    roS = 3;
    roD = 500;

    opacity = 255;
    r_rotS = radians(1);
    r_rotD = 5;
    max_ang_accel = 50;
    max_rotation  = 0.9;
    time_to_target = 1.1;
    time_to_target_rot = 0.6;
  
    scale = 0.4;
 
  }

  void run(){

    seek();
    //moveRandom();
  
  }


  void seek(){
    
    check_border();
    end_time = millis();
    elapsed_time = end_time - start_time;

    //breadcrumb_trail();
    elapsed_time = elapsed_time/100;
    orient();
    moveBoid();
    start_time = millis();
    
  }


}
