function [Ref] = GeraReferencia(varargin)

%fonte: https://www.mathworks.com/videos/varargin-and-nargin-variable-inputs-to-a-function-97389.html

%argumentos
% 1 o - t
% 2 o - Ax
% 3 o - Bx
% 4 o - Cx
% 5 o - Ay
% 6 o - By
% 7 o - Cy
% 8 o - Az
% 9 o - Bz
% 10o - Cz

%mandar valor do t em que voce quer que saja calculada a referencia
%feito dessa forma para que nao precise recompilar bbg toda vez que desejar
%mudar a referencia. A referencia contante ou variavel é enviada
%iterativamente para a beagle bone

switch nargin
    case 0
        disp('Argumentos default')
        
        t = 0; %segundos
        Ax = 0; Bx = 0; Cx = 1;
        Ay = 0; By = 0; Cy = 1;
        Az = 0; Bz = 0; Cz = 1;        
        
    case 1
        disp('1 argumento')
        
        t = varargin{1};
        Ax = 0; Bx = 0; Cx = 1;
        Ay = 0; By = 0; Cy = 1;
        Az = 0; Bz = 0; Cz = 1;   
        
    case 2
        disp('2 argumento')
        
        t = varargin{1};
        Ax = varargin{2}; Bx = 0; Cx = 1;
        Ay = 0;           By = 0; Cy = 1;
        Az = 0;           Bz = 0; Cz = 1;
        
    case 3
        disp('3 argumentos')
        
        t = varargin{1};
        Ax = varargin{2}; Bx = varargin{3}; Cx = 1;
        Ay = 0;           By = 0;           Cy = 1;
        Az = 0;           Bz = 0;           Cz = 1;
        
       
        
    otherwise
        error('erro! Numero de argumentos acima do permitido')
        
end

x = Ax*t.^2 + Bx*t + Cx
y = Ay*t.^2 + By*t + Cy;
z = Az*t.^2 + Bz*t + Cz;


Ref = [x; y; z];
