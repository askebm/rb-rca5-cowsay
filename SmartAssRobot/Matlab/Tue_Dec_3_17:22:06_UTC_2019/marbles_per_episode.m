%%
%clear
%folder='Mon 02 Dec 2019 01:31:04 PM CET';
%i=1;
%ALPHA = 1:9;
%EPSILON = 1:9;
%for a = 1:9
% for e = 1:9
%	 a_str = num2str(a);
%	 e_str = num2str(e);
%   str = ['test_circle_episodes_10000_nodes_20_lambda_0.7_epsilon_',e_str,'_alpha_',a_str,'_freq_50.csv'];
%	 data = load([folder,'/',str]);
%	 REWARD(a,e) = data(end);
%	 i = i + 1;
% end
%end
%
%
%surf(ALPHA/10,EPSILON/10,REWARD);
%xlabel('alpha');
%ylabel('epsilon');


%test_circle_episodes_10000_nodes_20_lambda_0.7_epsilon_1_alpha_1_freq_50.csv

clear
folder='TueDec3/';
i=1;
ALPHA = 10:2:100;
EPSILON = 10:2:100;
ia=1;
for a = 10:2:100
 ie=1;
 for e = 10:2:100
	 a_str = num2str(a);
	 e_str = num2str(e);
   str = ['test_circle_episodes_10000_nodes_20_lambda_70_epsilon_',e_str,'_alpha_',a_str,'_freq_100.csv'];
	 data = load([folder,str]);
	 REWARD(ia,ie) = data(end);
	 ie = ie + 1;
 end
 ia = ia + 1;
end


figure('Name','float_me','Position',[ 0 0 400 300 ])
surf(ALPHA/100,EPSILON/100,REWARD);
xlabel('$\alpha$');
ylabel('$\epsilon$');
view(45,45);
Plot2LaTeX(gcf,'CircleTestSideView')

figure('Name','float_me','Position',[ 0 0 400 300 ])
surf(ALPHA/100,EPSILON/100,REWARD);
xlabel('$\alpha$');
ylabel('$\epsilon$');
view(2);
Plot2LaTeX(gcf,'CircleTestTopView')
%%
