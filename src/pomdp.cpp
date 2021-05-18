#include "ros/ros.h"

#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <thread>

#include <AIToolbox/POMDP/Model.hpp>
#include <AIToolbox/MDP/Model.hpp>

#include <AIToolbox/POMDP/Algorithms/IncrementalPruning.hpp>
#include <AIToolbox/POMDP/Policies/Policy.hpp>
#include <AIToolbox/Impl/CassandraParser.hpp>
#include <fstream>

using namespace std;
using namespace AIToolbox::Impl;
using namespace AIToolbox::POMDP;


int main() {

	CassandraParser cp;

  std::default_random_engine rand(AIToolbox::Impl::Seeder::getSeed());


	string ln;
	
	std::ifstream modelFile;
	modelFile.open( "/home/jcacace/tiger.POMDP");
  if (modelFile.is_open()) {
		//	auto model = cp.parsePOMDP(modelFile);
		const auto & [S, A, O, T, R, W, discount] = cp.parsePOMDP(modelFile);

    AIToolbox::POMDP::Model<AIToolbox::MDP::Model> model(O, S, A);
    model.setTransitionFunction(T);
    model.setRewardFunction(R);
    model.setObservationFunction(W);
    
    model.setDiscount( discount );
    
     // Set the horizon. This will determine the optimality of the policy
    // dependent on how many steps of observation/action we plan to do. 1 means
    // we're just going to do one thing only, and we're done. 2 means we get to
    // do a single action, observe the result, and act again. And so on.
    unsigned horizon = 15;
    // The 0.0 is the tolerance factor, used with high horizons. It gives a way
    // to stop the computation if the policy has converged to something static.
    AIToolbox::POMDP::IncrementalPruning solver(horizon, 0.0);
		
    // Solve the model. After this line, the problem has been completely
    // solved. All that remains is setting up an experiment and see what
    // happens!
    auto solution = solver(model);

    // We create a policy from the solution, in order to obtain actual actions
    // depending on what happens in the environment.
    AIToolbox::POMDP::Policy policy(2, 3, 2, std::get<1>(solution));
    
    //cout << policy << endl;

    // We begin a simulation, we start from a uniform belief, which means that
    // we have no idea on which side the tiger is in. We sample from the belief
    // in order to get a "real" state for the world, since this code has to
    // both emulate the environment and control the agent. The agent won't know
    // the sampled state though, it will only have the belief to work with.
    AIToolbox::POMDP::Belief b(2); 
    b << 0.5, 0.5;
   	auto s = AIToolbox::sampleProbability(2, b, rand);

    // The first thing that happens is that we take an action, so we sample it now.
    auto [a, ID] = policy.sampleAction(b, horizon);
		
    // Setup cout to pretty print the simulation.
    std::cout.setf(std::ios::fixed, std::ios::floatfield);
    std::cout.precision(6);

    double totalReward = 0.0;
    int t = horizon - 1;
    for (int t = horizon - 1; t >= 0; --t) {
		//while( totalReward < 15 ) {
		  auto [s1, o, r] = model.sampleSOR(s, a);
      totalReward += r;			   
    
			b = AIToolbox::POMDP::updateBelief(model, b, a, o);
			if (t > (int)policy.getH())
				std::tie(a, ID) = policy.sampleAction(b, policy.getH());
			else
				std::tie(a, ID) = policy.sampleAction(ID, o, t);

			cout << "a: " << a << " - " << ID << endl;
			cout << "totalReward: " << totalReward << endl;
			// Then we update the world
			s = s1;
			//t--;
			// Sleep 1 second so the user can see what is happening.
			getline(cin, ln);
    }    
		
	}
	else
		cout << "Could not open model file!" << endl;

	


	return 0;
	
	
	
}
