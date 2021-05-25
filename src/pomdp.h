#include <AIToolbox/POMDP/Algorithms/IncrementalPruning.hpp>
#include <AIToolbox/POMDP/Policies/Policy.hpp>
#include <AIToolbox/MDP/Policies/Policy.hpp>
#include <AIToolbox/Impl/CassandraParser.hpp>

#include <AIToolbox/POMDP/Types.hpp>
#include <AIToolbox/POMDP/TypeTraits.hpp>
#include <AIToolbox/POMDP/IO.hpp>
#include <AIToolbox/POMDP/Utils.hpp>
#include <AIToolbox/Impl/CassandraParser.hpp>
#include <ros/package.h>
#include <fstream>
#include "boost/thread.hpp"

using namespace std;
using namespace AIToolbox::Impl;
using namespace AIToolbox::POMDP;
using namespace AIToolbox;


class POMDP_DEC {
    public:
        POMDP_DEC();
        void run();
        void init();
        void loop();
        void main_loop();

    private:

        ros::NodeHandle _nh;
        bool _load_from_file;
        bool _train_policy;
        string _model_file;        //.POMDP
        string _policy_file;      //.txt
        int _horizon;

        AIToolbox::POMDP::Policy *_policy;

        int _actions;
        int _states;
        int _observations;
        AIToolbox::POMDP::Model<AIToolbox::MDP::Model> *_model;
};