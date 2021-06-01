#include "ros/ros.h"

#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <thread>

#include <AIToolbox/POMDP/Model.hpp>
#include <AIToolbox/MDP/Model.hpp>

#include "pomdp.h"



POMDP_DEC::POMDP_DEC() {

  if( !_nh.getParam("load_from_file", _load_from_file) ) {
    _load_from_file =  false;
  }
  if( !_nh.getParam("train_policy", _train_policy) ) {
    _train_policy =  false;
  }
  if( !_nh.getParam("train_policy", _train_policy) ) {
    _train_policy =  false;
  }
  if( !_nh.getParam("model_file", _model_file) ) {
    _model_file = "Problema.POMDP";
  }
  if( !_nh.getParam("policy_file", _policy_file) ) {
    _policy_file = "policy.txt";
  }
  if( !_nh.getParam("horizon", _horizon) ) {
    _horizon = 10;
  }
  if( !_nh.getParam("states", _states) ) {
    _states = 576;
  }

  if( !_nh.getParam("actions", _actions) ) {
    _actions = 6;
  }

  if( !_nh.getParam("observations", _observations) ) {
    _observations = 384;
  }
  
}

void POMDP_DEC::init() {

  if( _load_from_file ) {
    
  
  }
  else if ( _train_policy ) {
    //TODO
    CassandraParser cp;
    string dir_path = ros::package::getPath("pomdp_dec");
    cout << "dir_path: " << dir_path << endl;
    std::ifstream modelFile;
    modelFile.open( dir_path + "/POMDP/" + _model_file);


    cout << "POMDP: " << dir_path + "/POMDP/" + _model_file << endl;
    const auto & [S, A, O, T, R, W, discount] = cp.parsePOMDP(modelFile);


    _model = new AIToolbox::POMDP::Model<AIToolbox::MDP::Model>(O,S,A);
    _model->setTransitionFunction(T);
    _model->setRewardFunction(R);
    _model->setObservationFunction(W);

    _model->setDiscount( discount );



    

  }
  else {
    ROS_ERROR("Impossible to calculate the policy");
  }
}



void POMDP_DEC::main_loop () {


		//aspetta che arrivi lo stato iniziale
		
  	ros::Rate srate(5);
  	AIToolbox::POMDP::IncrementalPruning solver(_horizon, 0.0);
  	auto solution = solver( *_model ); 
		AIToolbox::POMDP::Policy policy (_states, _actions, _observations, std::get<1>(solution) );
	
		//cout << policy << endl;
  	//Wait for initial state


  	AIToolbox::POMDP::Belief b(_states); 
 	  //b << 0.5, 0.5;
    for(int i=0; i<576; i++ ) b[i] = 0;
    	b[487] = 1.0;
    std::default_random_engine rand(AIToolbox::Impl::Seeder::getSeed());
   	auto s = AIToolbox::sampleProbability(_states, b, rand);

    auto [a, ID] = policy.sampleAction(b, _horizon);
	  auto [s1, o, r] = _model->sampleSOR(s, a);

    
    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(6);

    double totalReward = 0.0;
    int t = _horizon - 1;
    
    //for (int t = horizon - 1; t >= 0; --t) {
		//while( totalReward < 15 ) {
		
			//t--;
			// Sleep 1 second so the user can see what is happening.
			//getline(cin, ln);
    //}    

  while (ros::ok() ) {
    auto [s1, o, r] = _model->sampleSOR(s, a);
    totalReward += r;			   

    b = AIToolbox::POMDP::updateBelief( *_model, b, a, o);
    if (t > (int)policy.getH())
    	std::tie(a, ID) = policy.sampleAction(b, policy.getH());
    else
    	std::tie(a, ID) = policy.sampleAction(ID, o, t);

    cout << "a: " << a << " - " << ID << endl;
    cout << "totalReward: " << totalReward << endl;
    // Then we update the world
    s = s1;
   
    srate.sleep();
  }
}


void POMDP_DEC::run() {
  init();
  boost::thread main_loop_t( &POMDP_DEC::main_loop, this );
  ros::spin();
}

int main(int argc, char ** argv ) {

  ros::init(argc, argv, "pomdpdec" );

  POMDP_DEC pd;
  pd.run();

  return 0;


	
	
	
}
