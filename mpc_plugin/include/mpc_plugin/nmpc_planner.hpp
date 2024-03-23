#ifndef NMPC_PLANNER__NMPC_PLANNER_PLUGIN_HPP_
#define NMPC_PLANNER__NMPC_PLANNER_PLUGIN_HPP_

#include "mpc_plugin/mpc_base.hpp"
#include <mpc/NLMPC.hpp>

namespace nmpc_planner
{
class NMPCPlanner : public libmpc::LibMPCBase
	{
		public:

			void initialize() override;
			void stepController() override;
			void getOptimalStates(std::unique_ptr<Eigen::MatrixXd>&) override;
			void getOptimalInputs(std::unique_ptr<Eigen::MatrixXd>&) override;
			void setReference(const Eigen::VectorXd&) override;
			void setTrajectory(const Eigen::MatrixXd&) override;
			void setStateInput(const Eigen::VectorXd&) override;

			//variables
			const static int num_states = 4;
			const static int num_output = 2;
			const static int num_inputs = 2;
			const static int pred_hor = 20;
			const static int ctrl_hor = 20;
			const static int ineq_c = (pred_hor + 1);
			const static int eq_c = 0;
			double ts = 0.2;
			mpc::cvec<4> yref;
			mpc::mat<21, 2> trajxy;
			mpc::NLMPC<
    					num_states, num_inputs, num_output,
    					pred_hor, ctrl_hor,
    					ineq_c, eq_c> controller;
			mpc::cvec<num_states> modelX, modeldX;
							
	};
}

#endif //NMPC_PLANNER__NMPC_PLANNER_PLUGIN_HPP_

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nmpc_planner::NMPCPlanner, libmpc::LibMPCBase)