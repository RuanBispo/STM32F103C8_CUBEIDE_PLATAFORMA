function [ tamos,y,u,t ] = Update_data( tamos,y,u,t,flag_plot )

% Função que atualiza os dados de 'tamos', 'y', 'u' e 't'; atualiza a janela
% de observação para o plot (gráfica); plota os dados de y e u; e fixa o 
% periodo de amostragem do loop, o deixando constante. 

%%%%% Atualização de variáveis (tamos,y,u,t) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t(end) = toc;            % atualização do tempo em segundos
    tamos(end) = t(end) - t(end-1);
    
    tamos(1:end-1) = tamos(2:end); 
    t(1:end-1) = t(2:end);   % Vetores responsáveis pela visualização em
    y(1:end-1) = y(2:end);   % janela, assim a última amostra é deslocada
    u(1:end-1) = u(2:end);   % para a esquerda dando impressão de movimento

    
%%%%% Plots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    if(flag_plot)
        segundos = 0.1;                   %plota a cada x segundos
        if (mod(t(end),segundos) < 0.01)   %limita a quantidade de plots
%             subplot(211)
            plot(t,y,'r');                %plot saída do sistema   
            
%             subplot(212)                %plot da variável de controle
%             plot(t,u,'b');
            grid on                       %adicionar grade a visualização
            axis([min(t) max(t) -33000 33000])  %configurar eixos de visualização
            drawnow
        end
    end
        
%%%%% Fixação do período de amostragem %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     p_amos = 0.01;                    % periodo de amostragem em segundos
%     while(toc < (t(end-1) + p_amos))  % fixa o periodo de amostragem
%     end

end

