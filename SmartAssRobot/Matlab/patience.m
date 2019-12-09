%%
clear
folder='Wed_Dec_4_07:59:24_UTC_2019/';
i=1;
ALPHA = 10:2:101
EPSILON = 10:2:101
ia=1
for a = 10:2:101
 ie=1
 for e = 10:2:101
	 a_str = num2str(a);
	 e_str = num2str(e);
   str = ['test_patience_episodes_100000_nodes_9_lambda_70_epsilon_',e_str,'_alpha_',a_str,'_freq_100.csv'];
	 data = load([folder,str]);
	 REWARD(ia,ie) = data(end);
	 ie = ie + 1;
 end
 ia = ia + 1;
end


surf(ALPHA/100,EPSILON/100,REWARD);
xlabel('alpha');
ylabel('epsilon');

%%
