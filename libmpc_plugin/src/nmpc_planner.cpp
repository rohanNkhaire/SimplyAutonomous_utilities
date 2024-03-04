#include "libmpc_plugin/libmpc_base.hpp"
#include "libmpc_plugin/nmpc_planner.hpp"

namespace nmpc_planner
{
	NMPCPlanner::NMPCPlanner()
	{
	}

	void NMPCPlanner::initialize()
	{
		
		controller.setLoggerLevel(mpc::Logger::log_level::NORMAL);
  	controller.setContinuosTimeModel(ts);	
		auto stateEq = [&](
                   mpc::cvec<num_states> &dx,
                   const mpc::cvec<num_states> &x,
                   const mpc::cvec<num_inputs> &u)
    {
        dx(0) = x(3)*cos(x(2));
        dx(1) = x(3)*sin(x(2));
				dx(2) = u(0);
				dx(3) = u(1);
    };

		controller.setStateSpaceFunction([&](
                                        mpc::cvec<num_states> &dx,
                                        const mpc::cvec<num_states> &x,
                                        const mpc::cvec<num_inputs> &u,
                                        const unsigned int &)
                                    { stateEq(dx, x, u); });

    controller.setObjectiveFunction([&](
                                       const mpc::mat<pred_hor + 1, num_states> &x,
                                       const mpc::mat<pred_hor + 1, num_output> &,
                                       const mpc::mat<pred_hor + 1, num_inputs> &u,
                                       double)
                                   { 
																			double cost = 0;
        							for (int i = 0; i < (pred_hor + 1); ++i)
        							{
        							    cost += (x.row(i).segment(0, 2).transpose() - yref.segment(0, 2)).squaredNorm();
        							    cost += u.row(i).squaredNorm();
													cost += std::norm(x(i, 3) - yref(3));
													cost += std::norm(x(i, 2) - yref(2));
													cost += i > 0 ? std::norm(u(i) - u(i-1)) : 0.0;
											}
        							return cost;
											 });
	}

	void NMPCPlanner::stepController()
	{
		auto r = controller.getLastResult();
		r = controller.step(modelX, r.cmd);
	}

	Eigen::MatrixXd NMPCPlanner::getOptimalStates()
	{
		auto control_seq = controller.getOptimalSequence();
		return control_seq.state;
	}

	Eigen::MatrixXd NMPCPlanner::getOptimalInputs()
	{
		auto control_seq = controller.getOptimalSequence();
		return control_seq.input;
	}

} // namespace nmpc_planner

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(nmpc_planner::NMPCPlanner, libmpc::LibMPCBase)
