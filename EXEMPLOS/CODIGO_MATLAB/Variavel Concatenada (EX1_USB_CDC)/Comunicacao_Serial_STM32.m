clear
close all
clc

s = Config_USB();              % configuração do USB

%% INICIALIZAÇÃO DE VARIÁVEIS (y,u,t,tamos)
amost = 100;                   % amostras salvas no vetor de visualização
y = zeros(1,amost);            % recebida (saida do sistema)
t = zeros(1,amost); t(end)=0;  % tempo (300 é o tamanho da janela)
u = zeros(1,amost);            % enviada (variavel controlada)
imu = zeros(2,amost);          % recebida (saida do sistema
tamos = zeros(1,amost);        % período de amostragem (teste)
tic                            % inicio da contagem de tempo
parada=1;                      % flag de parada do while
flag_plot = 1;                 % se for 1, os dados são plotados

%% LAÇO PRINCIPAL
if(flag_plot), uicontrol('String','Parar','Callback','parada=0;'),end

while(parada)  
    % Comunicação USB [envia o u(t) e recebe o y(t)]
    fwrite(s,1,'uint8');
    
	% Recebimento de dados da IMU
	aux1 = fscanf(s,'%c');
	mpu = split(aux1,'+');
	imu(:,end) = str2double(mpu(1:2));
	
	%%%%% Atualização de variáveis (tamos,y,u,t) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	t(end) = toc;            % atualização do tempo em segundos
	tamos(end) = t(end) - t(end-1);

	tamos(1:end-1) = tamos(2:end); 
	t(1:end-1) = t(2:end);   % Vetores responsáveis pela visualização em
	imu(1,1:end-1) = imu(1,2:end);   % janela, assim a última amostra é deslocada
	imu(2,1:end-1) = imu(2,2:end);   % janela, assim a última amostra é deslocada

	%%%%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
	 segundos = 0.1;                   %plota a cada x segundos
	 if (mod(t(end),segundos) < 0.01)   %limita a quantidade de plots
		 plot(t,imu(1,:),'r');                %plot saída do sistema  
		 hold on
		 plot(t,imu(2,:),'b'); 
		 grid on                       %adicionar grade a visualização
		 axis([min(t) max(t) -185 185])  %configurar eixos de visualização
		 drawnow
	 end
    
    % Interface com o usuário: Command Window
%     CD_tamos_y_u = [tamos(end) y(end) u(end)];
    
    % Critério de parada - 1000 loops
    if(~flag_plot),parada=parada+1; if(parada==5000) ,parada=0;end,end

end

% Finalização de código %%%%%%%%%%%%%%%%%%%%
periodo_de_amostragem_medio = mean(tamos) %%
figure(2), plot(tamos)                    %%
fclose(s);                                %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%