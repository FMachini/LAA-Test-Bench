IP_Local = '127.0.0.1';
PORT = 8080; 
t = tcpclient(IP_Local, PORT);

dados = read(t, 2, 'double')
write(t, [16.12 19.93]);

clear t