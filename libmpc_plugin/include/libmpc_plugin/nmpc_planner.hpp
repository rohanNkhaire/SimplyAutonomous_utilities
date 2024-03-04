#include "libmpc_plugin/libmpc_base.hpp"

namespace nmpc_planner
{
class NMPCPlanner : public libmpc::LibMPCBase
	{
		public:
			NMPCPlanner();

			void initialize() override;
			void stepController() override;
			Eigen::MatrixXd getOptimalStates() override;
			Eigen::MatrixXd getOptimalInputs() override;

			//variables
			const static int num_states = 4;
			const static int num_output = 2;
			const static int num_inputs = 2;
			const static int pred_hor = 30;
			const static int ctrl_hor = 30;
			const static int ineq_c = 0;
			const static int eq_c = 0;
			double ts = 0.1;
			mpc::cvec<4> yref;
			mpc::NLMPC<
    					num_states, num_inputs, num_output,
    					pred_hor, ctrl_hor,
    					ineq_c, eq_c> controller;
			mpc::cvec<num_states> modelX, modeldX;
							
	};
}