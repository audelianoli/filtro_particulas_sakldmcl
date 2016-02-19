#include "filtro_particulas_sakldmcl.h"

int main (int argc, char** argv)
{
	ros::init( argc, argv, "filtro_particulas_sakldmcl_node" );

	ros::NodeHandle n;

	Filtro_Particulas_Sakldmcl fpsakldmcl(n);

	fpsakldmcl.spin();

	return 0;

}
