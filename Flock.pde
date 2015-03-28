
class Flock
{
  
  //instantiates multiple boids and flocking mechanisms
  ArrayList<FlockBoid> flock;
  FlockBoid boid = new FlockBoid(); 
  RandLeaders leaders;
  
  BoidLeader nearest_leader = new BoidLeader();
  PVector flock_centroid, dir_toleader, prev_centroid, centroid_velocity;
  float distance_toNearestLeader;
  //blending weights
  int arrive_weight = 15; //seek leader
  int sep_weight = 13; //collision avoidance
  int join_weight = 3; //cohesion as a group
  //int push_weight = 70; //simple collision avoidance
  int match_weight = 3; //match velocity of centroid
  int make_way_weight =15; //make way for the leader
  
  Flock(int n,int k){
    
    flock = new ArrayList<FlockBoid>(n);
    flock_centroid = new PVector(0,0);
    
    if (k == 0){
      k=1;
      }
    leaders = new RandLeaders(k); 
      
    for (int i = 0; i <= n; i++){ 
        
      flock.add(new FlockBoid());
        
      }
   
 }
  
  void run_flock(){
    
    leaders.make_leaders();
    nearest_leader = leaders.leader_list.get(0);
    
    for (int i = 0; i < flock.size(); i++) {
     
      boid = flock.get(i);
      prev_centroid = flock_centroid.get();
      flock_centroid = PVector.add(flock_centroid, boid.position);
      
      for (int j = 0; j < leaders.leader_list.size(); j++) {
        
        BoidLeader boid_leader = leaders.leader_list.get(j);
        dir_toleader = PVector.sub(nearest_leader.position,boid.position);
        distance_toNearestLeader = dir_toleader.mag(); 
        dir_toleader = PVector.sub(boid_leader.position,boid.position);
        
        float distance_toLeader = dir_toleader.mag(); 
        
        if (distance_toLeader < distance_toNearestLeader){
            
          distance_toNearestLeader = distance_toLeader;
          nearest_leader = boid_leader;
              
          }
          
              
        }

      PVector steer = new PVector(0,0);
      PVector arrive = boid.arrive(nearest_leader.position);
      arrive.mult(arrive_weight); //weight
      steer.add(arrive);
       
      //Flocking mechanisms  
      PVector sep = collision_avoidance(i);
      sep.mult(sep_weight); //weight
      steer.add(sep);
        
      //PVector push = separation(i);
      //push.mult(push_weight);
      //steer.add(push);
      
      flock_centroid.div(flock.size());
      centroid_velocity = PVector.sub(flock_centroid, prev_centroid);
   
        
      PVector join = check_cohesion();
      join.mult(join_weight); //weight
      steer.add(join);
      
      PVector match = match_velocity(centroid_velocity);
      match.mult(match_weight); //weight
      steer.add(match);
        
      PVector make_way = make_way();
      make_way.mult(make_way_weight); //weight
      steer.add(make_way);
        
      steer.normalize();
      steer.mult(boid.max_accel/3);
        
      boid.update(steer);
      
      boid.run();
 
      }
      
   }

  PVector collision_avoidance(int i){
    
    float t_min = 10000;
    PVector dp = new PVector(0,0);
    PVector rel_pos = new PVector(0,0);
    PVector rel_pos_top = new PVector(0,0);
    PVector dv = new PVector(0,0);
    PVector rel_vel_top = new PVector(0,0);
    PVector Pcd = new PVector(0,0);
    PVector Ptd = new PVector(0,0);
    PVector Pcd_top = new PVector(0,0);
    PVector Ptd_top = new PVector(0,0);
    float t_of_ca;
    
    FlockBoid top_priority = new FlockBoid();
    
    for (int j = 0; j < flock.size(); j++ ) {
      
      if (j != i){

        FlockBoid other = flock.get(j);
        flock_centroid = PVector.add(flock_centroid, other.position);
        PVector dist_other = PVector.sub(boid.position,other.position);
          
        dp = PVector.sub(boid.position,other.position);
        dv = PVector.sub(boid.velocity,other.velocity);
        t_of_ca = -((PVector.dot(dp, dv))/(dv.magSq()));
        if (t_of_ca < 0){continue;}        

        PVector vc_t = PVector.mult(boid.velocity,t_of_ca);
        Pcd = PVector.add(boid.position, vc_t);
         
        PVector vt_t = PVector.mult(other.velocity,t_of_ca);
        Ptd = PVector.add(other.position, vt_t);
         
        PVector Pcptdiff = PVector.sub(Pcd,Ptd);
        if (Pcptdiff.mag() < 35){
             //possible collision
         
          if (t_of_ca < t_min){
            
            t_min = t_of_ca;
            top_priority = other;
            rel_pos_top = dp;
            rel_vel_top = dv;
            Pcd_top = Pcd;
            Ptd_top = Ptd;
              
           }
             
         }
          
       }
      
     }
         
    if ((t_min == 0) || (Pcd_top == Ptd_top)){
      
      rel_pos = PVector.sub(boid.position, top_priority.position);
         
     }
    else { 
      
      PVector v_t = PVector.mult(rel_vel_top,t_min);
      rel_pos = PVector.add(rel_pos_top,v_t);
           
     }
    
    rel_pos.normalize();
    rel_pos.mult(boid.max_accel);
    return rel_pos;
         
  }

    
  PVector check_cohesion(){
    
    PVector dist_flock_centroid = PVector.sub(flock_centroid,boid.position);
    PVector cohesion_steering = new PVector(0,0);
    if (dist_flock_centroid.mag() > width/8){
      
      PVector goal_velocity = dist_flock_centroid;
      goal_velocity.normalize();
      goal_velocity.mult(boid.max_speed);
      cohesion_steering = PVector.sub(goal_velocity,boid.velocity);
      cohesion_steering.div(boid.time_to_target);
      
      if (cohesion_steering.mag() > boid.max_accel){
        
        cohesion_steering.normalize();
        cohesion_steering.mult(boid.max_accel);
  
       }    

    }
    
    return cohesion_steering;
    
  }
    
  PVector match_velocity(PVector centroid_velocity){
   
    PVector match_steering = PVector.sub(centroid_velocity, boid.velocity);
      match_steering = PVector.div(match_steering, boid.time_to_target);
      if (match_steering.mag() > boid.max_accel){
        
        match_steering.normalize();
        match_steering.mult(boid.max_accel);
        
       }
    
    return match_steering;
    
  }
  
    

//Experimental
  PVector separation(int i){
        
    PVector sum_sep_velocity = new PVector(0,0);
        
    for (int j = 0; j < flock.size(); j++ ) {
          
      if (j != i){
  
        FlockBoid other = flock.get(j);
        PVector dist_other = PVector.sub(boid.position,other.position);
        PVector sep_velocity = new PVector(0,0);
        float desired_seperation = 25;
          
        if (dist_other.mag() > 60){
          
          continue;
          
        }
          
        if (dist_other.mag() < desired_seperation){
      
          sep_velocity = dist_other.get();
          sep_velocity.normalize();
          sep_velocity.mult(boid.max_speed);
          //sep_velocity.limit(boid.max_speed);
          sum_sep_velocity = PVector.add(sum_sep_velocity,sep_velocity);
           
          }

      }
      
    }
      
    PVector goal_velocity = sum_sep_velocity;
    goal_velocity.normalize();
    goal_velocity.mult(boid.max_speed);
  
    PVector sep_steering = PVector.sub(goal_velocity,boid.velocity);
    sep_steering.div(boid.time_to_target);
  
    if (sep_steering.mag() > boid.max_accel){
      sep_steering.normalize();
      sep_steering.mult(boid.max_accel);
  
     }
      
    return sep_steering;
  
  }
    
    
    
  PVector make_way(){
  
    PVector posdist = PVector.sub(nearest_leader.position, boid.position);
    PVector flee_pos = new PVector(0,0);
    PVector flee_futpos = new PVector(0,0);
    PVector v_t = PVector.mult(nearest_leader.velocity, nearest_leader.elapsed_time*15);
    PVector futpos = PVector.add(nearest_leader.position,v_t); 
    PVector futposdist = PVector.sub(futpos, boid.position);      
    
    if (posdist.mag() < 30) {
    
      flee_pos = boid.flee(nearest_leader.position);
        
      }
        
    if (futposdist.mag() < 80) {

      flee_futpos = boid.flee(nearest_leader.position);
       
      }
        
    PVector make_way_steer = PVector.add(flee_pos, flee_futpos);
    return make_way_steer;
      
    }
    

}
          
    
    
    
    
    

 
   
