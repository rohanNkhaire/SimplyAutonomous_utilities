#ifndef LIBMPC_BASE_LIBMPC_BASE_HPP
#define LIBMPC_BASE_LIBMPC_BASE_HPP

#include <Eigen/Core>

namespace libmpc
{
	class LibMPCBase
	{
	public:
		virtual void initialize() = 0;
		virtual void stepController() = 0;
		virtual Eigen::MatrixXd getOptimalStates() = 0;
		virtual Eigen::MatrixXd getOptimalInputs() = 0;
		virtual void setReference(const Eigen::VectorXd&) = 0; 
		virtual void setStateInput(const Eigen::VectorXd&) = 0;
		virtual ~LibMPCBase(){};

	protected:
		LibMPCBase(){};	
	};
} // namespace libmpc


#endif // LIBMPC_BASE_LIBMPC_BASE_HPP