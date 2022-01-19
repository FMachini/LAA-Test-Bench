#include "Comm.hpp"

int main()
{

Comm comm;
while(true)
{
comm.accepting();

double v[2] = {20.08, 19.66};
double buffer[2];

comm.enviarDados(v, sizeof(v));
comm.receberDados(buffer, sizeof(buffer));
std::cout << "recebi: " << std::endl;
std::cout << buffer[0] << std::endl;
std::cout << buffer[1] << std::endl;

comm.closeSocket();
}
std::cout << "##EOP##" << std::endl;

return 0;
}
