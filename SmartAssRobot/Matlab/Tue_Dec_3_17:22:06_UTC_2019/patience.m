%%
clear
folder='Mon 02 Dec 2019 03:40:17 PM CET/';
i=1;
ALPHA = 1:9;
EPSILON = 1:9;
for a = 1:9
 for e = 1:9
	 a_str = num2str(a);
	 e_str = num2str(e);
   str = ['test_patience_episodes_1000000_nodes_7_lambda_0.7_epsilon_',e_str,'_alpha_',a_str,'_freq_50.csv'];
	 data = load([folder,str]);
	 REWARD(a,e) = data(end);
	 i = i + 1;
 end
end


surf(ALPHA/10,EPSILON/10,REWARD);
xlabel('alpha');
ylabel('epsilon');

%%
