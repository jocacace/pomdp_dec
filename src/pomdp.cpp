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
#include <AIToolbox/MDP/Policies/Policy.hpp>
#include <AIToolbox/Impl/CassandraParser.hpp>
#include <fstream>



/*

#include <AIToolbox/MDP/Environments/CliffProblem.hpp>

#include <AIToolbox/MDP/Experience.hpp>
#include <AIToolbox/MDP/MaximumLikelihoodModel.hpp>

#include <AIToolbox/MDP/Algorithms/QLearning.hpp>
#include <AIToolbox/MDP/Algorithms/PrioritizedSweeping.hpp>

#include <AIToolbox/MDP/Policies/QGreedyPolicy.hpp>
#include <AIToolbox/MDP/Policies/EpsilonPolicy.hpp>
*/
#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/POMDP/TypeTraits.hpp>
#include <AIToolbox/POMDP/IO.hpp>

#include <AIToolbox/POMDP/Utils.hpp>

#include <AIToolbox/Impl/CassandraParser.hpp>

using namespace std;
using namespace AIToolbox::Impl;
using namespace AIToolbox::POMDP;
using namespace AIToolbox;


 bool checkRemoveAtSign(std::istream &is) {
        char c = (is >> std::ws).peek();
        if ( c == '@' ) {
            is >> c;
            return true;
        }
        return false;
    }

int main() {

	CassandraParser cp;

  std::default_random_engine rand(AIToolbox::Impl::Seeder::getSeed());

	string ln;
  unsigned horizon = 5;

	std::ifstream modelFile;
	modelFile.open( "/home/jcacace/big.POMDP");
  //if (modelFile.is_open()) {
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
    // The 0.0 is the tolerance factor, used with high horizons. It gives a way
    // to stop the computation if the policy has converged to something static.
    AIToolbox::POMDP::IncrementalPruning solver(horizon, 0.0);
		
    // Solve the model. After this line, the problem has been completely
    // solved. All that remains is setting up an experiment and see what
    // happens!
    auto solution = solver(model);

    // We create a policy from the solution, in order to obtain actual actions
    // depending on what happens in the environment.
    //AIToolbox::POMDP::Policy policy(2, 3, 2, std::get<1>(solution));
    AIToolbox::POMDP::Policy policy(576, 6, 384, std::get<1>(solution));
    


    //ofstream f;
    //f.open("/tmp/policy.txt",  std::ofstream::out | std::ofstream::app);
    //file << &policy;
    //std::cout << &policy << std::endl;
    auto & vf = policy.getValueFunction();
    
    // VLists
    for ( size_t h = 1; h < vf.size(); ++h ) {
      const auto & vl = vf[h];
      // VEntries
      
      for ( auto & vv : vl ) {
        // Values
        cout << vv.values.transpose() << ' ';
        // Action
        cout << vv.action << ' ';
        // Obs
        for ( const auto & o : vv.observations )
          cout << o << ' ';
          cout << '\n';
        }
        // Horizon separator
        cout << "@\n";
    
    }
      // We close with a second at sign so that other things can also be
      // put on the stream, and the loader will work.
      cout << "@\n";    
  //}



    AIToolbox::POMDP::Policy policy2(576, 6, 384);
    const size_t Sa = policy2.getS();
    const size_t Aa = policy2.getA();
    const size_t Oa = policy2.getO();
 
    auto vfa = makeValueFunction(Sa);
    size_t oldH = 1;
    bool newHorizon = true;
//exit(0);
/*

    std::ifstream fb;
    fb.open ("/tmp/policy",std::ios::in);

    fb >> policy2;

    */
/*
    bool done = false;
    while ( !done ) {
      if ( newHorizon ) {
        // If we find a '@' here, we have finished.
        if ( checkRemoveAtSign(is) ) {
          done = true;
          continue;          
        }

        oldH = vfa.back().size();
        vfa.emplace_back();
        newHorizon = false;
      }
      MDP::Values values(S);
      size_t action;
      POMDP::VObs obs(O, 0);

      // Values
      for ( size_t i = 0; i < S; ++i )
        if ( !(is >> values(i)) )
          //goto failure;
          exit(0);
          // Action
        if ( !(is >> action) || action >= A ) {
          exit(0);
        }
        // Obs
        for ( auto & o : obs ) {
          if ( !(is >> o) || ( o >= oldH && oldH ) ) {
            exit(0);//goto failure;
          }
        }

        vfa.back().emplace_back(std::move(values), action, std::move(obs));

        // Check if next char after whitespace is a @ that
        // marks a new horizon.
        if ( checkRemoveAtSign(is) )
          newHorizon = true;
      }

      policy2.H = vfa.size() - 1;
      //p.policy_ = std::move(vf);
    // We begin a simulation, we start from a uniform belief, which means that
    // we have no idea on which side the tiger is in. We sample from the belief
    // in order to get a "real" state for the world, since this code has to
    // both emulate the environment and control the agent. The agent won't know
    // the sampled state though, it will only have the belief to work with.
    */
    AIToolbox::POMDP::Belief b(576); 
    //b << 0.5, 0.5;
    for(int i=0; i<576; i++ ) b[i] = 0;
    b[487] = 1.0;

   	auto s = AIToolbox::sampleProbability(576, b, rand);

    // The first thing that happens is that we take an action, so we sample it now.
    
    //auto s = b;
    //State s(576); 
cout << "Loaded policy!" << endl;
    auto [a, ID] = policy.sampleAction(b, horizon);
	  auto [s1, o, r] = model.sampleSOR(s, a);

    cout << "Action: " << a << " - " << ID << " - " << r  << endl;
    /*
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
			//getline(cin, ln);
    }    
		
	}
	else
		cout << "Could not open model file!" << endl;

    */
    /*
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
			//getline(cin, ln);
    }    
		
	}
	else
		cout << "Could not open model file!" << endl;

  */
	return 0;
	
	
	
}
