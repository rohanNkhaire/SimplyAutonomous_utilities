#include "mpc_plugin/nmpc_planner.hpp"

namespace nmpc_planner
{

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
				dx(2) = u(1);
				dx(3) = u(0);
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
													// State cost
        							    cost += 1*(x.row(i).segment(0, 2).transpose() - yref.segment(0, 2)).squaredNorm();
													
													if (i == pred_hor)
													{
														cost += (x.row(i).segment(2, 1).transpose() - yref.segment(2, 1)).squaredNorm();
													}
													
													//cost += (x.row(i).segment(3, 1).transpose() - yref.segment(3, 1)).squaredNorm();
													// Input cost
													cost += 0.05*u.row(i).squaredNorm();
													cost += i > 0 ? 0.11*(u.row(i)-u.row(i-1)).squaredNorm() : 0.1*u.row(i).squaredNorm();
													// traj tracking lateral cost
													cost += 1*(x.row(i).segment(0,2).transpose() - trajxy.row(i).segment(0,2).transpose()).squaredNorm();
													cost += 1000*(x.row(i).segment(2,1).transpose() - trajxy.row(i).segment(2,1).transpose()).squaredNorm();
											}
        							return cost;
											 });

		controller.setIneqConFunction([&](
                                     mpc::cvec<ineq_c> &in_con,
                                     const mpc::mat<pred_hor + 1, num_states> &,
                                     const mpc::mat<pred_hor + 1, num_output> &,
                                     const mpc::mat<pred_hor + 1, num_inputs> &u,
                                     const double &)
                                 {
        for (int i = 0; i < ineq_c; i++) {
            in_con(i) = u(i, 0) - 1.0;
        } });

	}

	void NMPCPlanner::stepController()
	{
		mpc::Result<2> r;
		r = controller.step(modelX, r.cmd);
		std::cout << r.cmd << "\n";
	}

	void NMPCPlanner::setReference(const Eigen::VectorXd& ref)
	{
		yref = ref;
		std::cout << yref(0) << "\n";
		std::cout << yref(1) << "\n";
		std::cout << yref(2) << "\n";
		std::cout << yref(3) << "\n";
	}

	void NMPCPlanner::setTrajectory(const Eigen::MatrixXd& traj_mat)
	{
		trajxy = traj_mat;
	}

	void NMPCPlanner::setStateInput(const Eigen::VectorXd& input_state)
	{
		modelX = input_state;
		std::cout << modelX(0) << "\n";
		std::cout << modelX(1) << "\n";
		std::cout << modelX(2) << "\n";
		std::cout << modelX(3) << "\n";
	}

	void NMPCPlanner::getOptimalStates(std::unique_ptr<Eigen::MatrixXd>& opt_states)
	{
		auto control_seq = controller.getOptimalSequence();
		(*opt_states) = control_seq.state;
	}

	void NMPCPlanner::getOptimalInputs(std::unique_ptr<Eigen::MatrixXd>& opt_inputs)
	{
		auto control_seq = controller.getOptimalSequence();
		(*opt_inputs) = control_seq.input;

	}

} // namespace nmpc_planner
