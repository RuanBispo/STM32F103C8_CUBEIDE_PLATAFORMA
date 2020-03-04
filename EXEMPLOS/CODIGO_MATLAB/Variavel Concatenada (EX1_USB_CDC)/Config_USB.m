function [ s ] = Config_USB(  )

% Função que configura toda a porta serial utilizada pelo uC e corrige os
% erros que são gerados por remoção da porta e não fechamento da COM. 
% É necessário alterar o número da COMx toda vez que alterar o computador,
% para conferir a porta utilizada, ir em gerenciador de dispositivos

%%% CONFIGURAÇÃO DA PORTA SERIAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Caso não acesse é porque nao foi fechado
if isempty(instrfind) ~= 1 %ou seja, se não estiver vazio
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

