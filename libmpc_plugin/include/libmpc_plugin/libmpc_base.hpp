#ifndef LIBMPC_BASE_LIBMPC_BASE_HPP
#define LIBMPC_BASE_LIBMPC_BASE_HPP

#include <complex>
#include <Eigen/Core>

namespace libmpc
{
	class LibMPCBase
	{
	public:
		virtual void initialize();
		virtual void stepController();
		virtual Eigen::MatrixXd getOptimalStates() = 0;
		virtual Eigen::MatrixXd getOptimalInputs() = 0;
		virtual void setReference(const Eigen::VectorXd&); 
		virtual void setStateInput(const Eigen::VectorXd&);
		virtual ~LibMPCBase(){};

	protected:
		LibMPCBase(){};	
	};
} // namespace libmpc


#endif // LIBMPC_BASE_LIBMPC_BASE_HPP