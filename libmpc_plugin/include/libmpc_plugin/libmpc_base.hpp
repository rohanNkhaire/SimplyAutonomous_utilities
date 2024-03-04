#ifndef LIBMPC_BASE_LIBMPC_BASE_HPP
#define LIBMPC_BASE_LIBMPC_BASE_HPP

#include <mpc/NLMPC.hpp>
#include <mpc/LMPC.hpp>
#include <complex>
#include <Eigen/Core>

namespace libmpc
{
	class LibMPCBase
	{
	public:
		virtual void initialize();
		virtual void stepController();
		virtual Eigen::MatrixXd getOptimalStates();
		virtual Eigen::MatrixXd getOptimalInputs(); 
		virtual ~LibMPCBase(){};

	protected:
		LibMPCBase(){};	
	};
} // namespace libmpc


#endif // LIBMPC_BASE_LIBMPC_BASE_HPP