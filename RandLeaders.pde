
class RandLeaders
{
  //creates n boid leaders
  BoidLeader boid_leader = new BoidLeader();
  ArrayList<BoidLeader> leader_list;
  
  RandLeaders(int n){
  
    leader_list = new ArrayList<BoidLeader>(n);
    
    for (int i = 0; i < n; i++){ 
      
      leader_list.add(new BoidLeader());
      }
  
  }
  
  void make_leaders(){

    for (int i = 0; i < leader_list.size(); i++) {

      boid_leader = leader_list.get(i);
      boid_leader.run();

    }

  }
  
}
