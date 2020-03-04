function [ s ] = Config_USB(  )

% Fun��o que configura toda a porta serial utilizada pelo uC e corrige os
% erros que s�o gerados por remo��o da porta e n�o fechamento da COM. 
% � necess�rio alterar o n�mero da COMx toda vez que alterar o computador,
% para conferir a porta utilizada, ir em gerenciador de dispositivos

%%% CONFIGURA��O DA PORTA SERIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Caso n�o acesse � porque nao foi fechado
if isempty(instrfind) ~= 1 %ou seja, se n�o estiver vazio
    fclose(instrfind);
    delete(instrfindall);
end

% Nome da porta Serial a ser Conectada
s = serial('COM18');       %varia de acordo com o compuador
set(s,'BaudRate',57600);
set(s,'DataBits',8);
set(s,'Parity','None');
set(s,'stopBits',1);
set(s,'FlowControl','none');
set(s,'InputBufferSize',100);
set(s,'OutputBufferSize',100);
set(s,'Timeout',0.001);
set(s,'Terminator','LF');

%Abrindo a porta serial
fopen(s);
flushinput(s);
flushoutput(s);


end

