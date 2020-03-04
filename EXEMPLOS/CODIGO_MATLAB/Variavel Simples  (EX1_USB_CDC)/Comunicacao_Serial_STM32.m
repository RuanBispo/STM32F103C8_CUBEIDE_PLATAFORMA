clear
close all
clc

s = Config_USB();              % configura��o do USB

%% INICIALIZA��O DE VARI�VEIS (y,u,t,tamos)
amost = 200;                   % amostras salvas no vetor de visualiza��o
y = zeros(1,amost);            % recebida (saida do sistema)
t = zeros(1,amost); t(end)=0;  % tempo (300 � o tamanho da janela)
u = zeros(1,amost);            % enviada (variavel controlada)
tamos = zeros(1,amost);        % per�odo de amostragem (teste)
tic                            % inicio da contagem de tempo
parada=1;                      % flag de parada do while
flag_plot = 1;                 % se for 1, os dados s�o plotados

%% LA�O PRINCIPAL
if(flag_plot), uicontrol('String','Parar','Callback','parada=0;'),end

while(parada)
    
    % atualiza��o de variaveis e plots
    [tamos,y,u,t] = Update_data(tamos,y,u,t,flag_plot);
    
    % Comunica��o USB [envia o u(t) e recebe o y(t)]
    fwrite(s,u(end),'uint8');
    y(end) = fscanf(s,'%f'); 
    
    % Interface com o usu�rio: Command Window
    CW_tamos_y_u = [tamos(end) y(end) u(end)];
    
    % Crit�rio de parada - 1000 loops
    if(~flag_plot),parada=parada+1; if(parada==1000) ,parada=0;end,end

end

% Finaliza��o de c�digo %%%%%%%%%%%%%%%%%%%%
periodo_de_amostragem_medio = mean(tamos) %%
figure(2), plot(tamos)                    %%
fclose(s);                                %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%