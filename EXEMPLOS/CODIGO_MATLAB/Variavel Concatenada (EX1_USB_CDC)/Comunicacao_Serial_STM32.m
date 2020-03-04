clear
close all
clc

s = Config_USB();              % configura��o do USB

%% INICIALIZA��O DE VARI�VEIS (y,u,t,tamos)
amost = 100;                   % amostras salvas no vetor de visualiza��o
y = zeros(1,amost);            % recebida (saida do sistema)
t = zeros(1,amost); t(end)=0;  % tempo (300 � o tamanho da janela)
u = zeros(1,amost);            % enviada (variavel controlada)
imu = zeros(2,amost);          % recebida (saida do sistema
tamos = zeros(1,amost);        % per�odo de amostragem (teste)
tic                            % inicio da contagem de tempo
parada=1;                      % flag de parada do while
flag_plot = 1;                 % se for 1, os dados s�o plotados

%% LA�O PRINCIPAL
if(flag_plot), uicontrol('String','Parar','Callback','parada=0;'),end

while(parada)  
    % Comunica��o USB [envia o u(t) e recebe o y(t)]
    fwrite(s,1,'uint8');
    
	% Recebimento de dados da IMU
	aux1 = fscanf(s,'%c');
	mpu = split(aux1,'+');
	imu(:,end) = str2double(mpu(1:2));
	
	%%%%% Atualiza��o de vari�veis (tamos,y,u,t) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	t(end) = toc;            % atualiza��o do tempo em segundos
	tamos(end) = t(end) - t(end-1);

	tamos(1:end-1) = tamos(2:end); 
	t(1:end-1) = t(2:end);   % Vetores respons�veis pela visualiza��o em
	imu(1,1:end-1) = imu(1,2:end);   % janela, assim a �ltima amostra � deslocada
	imu(2,1:end-1) = imu(2,2:end);   % janela, assim a �ltima amostra � deslocada

	%%%%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
	 segundos = 0.1;                   %plota a cada x segundos
	 if (mod(t(end),segundos) < 0.01)   %limita a quantidade de plots
		 plot(t,imu(1,:),'r');                %plot sa�da do sistema  
		 hold on
		 plot(t,imu(2,:),'b'); 
		 grid on                       %adicionar grade a visualiza��o
		 axis([min(t) max(t) -185 185])  %configurar eixos de visualiza��o
		 drawnow
	 end
    
    % Interface com o usu�rio: Command Window
%     CD_tamos_y_u = [tamos(end) y(end) u(end)];
    
    % Crit�rio de parada - 1000 loops
    if(~flag_plot),parada=parada+1; if(parada==5000) ,parada=0;end,end

end

% Finaliza��o de c�digo %%%%%%%%%%%%%%%%%%%%
periodo_de_amostragem_medio = mean(tamos) %%
figure(2), plot(tamos)                    %%
fclose(s);                                %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%