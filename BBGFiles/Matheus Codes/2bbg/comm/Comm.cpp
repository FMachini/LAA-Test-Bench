#include "Comm.hpp"

Comm::Comm()
{
    createSocket();
    bindSetup();
    binding();
    listening();
}

Comm::~Comm()
{
	closeSocket();
}

void Comm::createSocket()
{
	if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
		{
			perror("Socket's creation failed");
			exit(EXIT_FAILURE);
		}
		
	// Forcefully attaching socket to the port 8080
	if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, (char* ) &opt, sizeof(opt)))
	{
		perror("Setsockopt");
		exit(EXIT_FAILURE);
	}
}

void Comm::bindSetup()
{
	address.sin_family = AF_INET; //IPv4
	address.sin_addr.s_addr = INADDR_ANY; //localhost
	address.sin_port = htons(PORT); //port
}

void Comm::binding()
{
	// Forcefully attaching socket to the port 8080
	if(bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0)
	{
			perror("bind failed");
			exit(EXIT_FAILURE);
	}
}

void Comm::listening()
{
	if(listen(server_fd, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
}

void Comm::accepting()
{
	if((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0)
		{
			perror("accept");
			exit(EXIT_FAILURE);
		}
        std::cout << "Connection stablished." << std::endl;

}

void Comm::receberDados(void* Guardar, size_t lenGuardar)
{
	recv(new_socket, Guardar, lenGuardar, 0);
}

void Comm::enviarDados(void* Enviar, size_t lenEnviar)
{
	send(new_socket, Enviar, lenEnviar, 0);
}

void Comm::closeSocket()
{
        int ec;
        ec = close(new_socket);
        if(ec == 0) std::cout << "Closing connection with the client." << std::endl;
        else std::cout << "error closing the socket." << std::endl;
        
}

void Comm::printData(const char *label, double *vetor, size_t sizeVet)
{
    std::cout << label << ": " << std::endl;
    
    for(int i = 0; i < sizeVet; i++)
    {
        std::cout << i << ": " << *(vetor++) << std::endl;
    }

    std::cout << "-----------------------" << std::endl;

}
