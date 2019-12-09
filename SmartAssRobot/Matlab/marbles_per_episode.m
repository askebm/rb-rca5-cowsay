%%
clear
folder='Tue_Dec_3_17:22:06_UTC_2019/';
i=1;
ALPHA = 10:2:101;
EPSILON = 10:2:101;
ia=1;
for a = 10:2:101
 ie=1;
 for e = 10:2:101
	 a_str = num2str(a);
	 e_str = num2str(e);
   str = ['test_circle_episodes_100000_nodes_20_lambda_70_epsilon_',e_str,'_alpha_',a_str,'_freq_500.csv'];
	 data = load([folder,str]);
	 REWARD(ia,ie) = data(end);
	 ie = ie + 1;
 end
 ia = ia + 1;
end



%figure('Name','float_me','Position',[ 0 0 400 300 ])
surf(ALPHA/100,EPSILON/100,REWARD);
xlabel('$\alpha$');
ylabel('$\epsilon$');
%view(45,45);
%Plot2LaTeX(gcf,'CircleTestSideView100000')
%
%figure('Name','float_me','Position',[ 0 0 400 300 ])
%surf(ALPHA/100,EPSILON/100,REWARD);
%xlabel('$\alpha$');
%ylabel('$\epsilon$');
%view(2);
%Plot2LaTeX(gcf,'CircleTestTopView100000')
%%
