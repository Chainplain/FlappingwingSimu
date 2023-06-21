rng('default') % For reproducibility
x_observed = linspace(0,10,21)';
y_observed1 = x_observed.*sin(x_observed);
y_observed2 = y_observed1 + 0.5*randn(size(x_observed));

gprMdl1 = fitrgp(x_observed,y_observed1);
gprMdl2 = fitrgp(x_observed,y_observed2);



x = linspace(0,10)';
[ypred1,~,yint1] = predict(gprMdl1,x);
[ypred2,~,yint2] = predict(gprMdl2,x);

fig = figure;
fig.Position(3) = fig.Position(3)*2;

tiledlayout(1,2,'TileSpacing','compact');

nexttile
hold on
scatter(x_observed,y_observed1,'r') % Observed data points
fplot(@(x) x.*sin(x),[0,10],'--r')  % Function plot of x*sin(x)
plot(x,ypred1,'g')                  % GPR predictions
patch([x;flipud(x)],[yint1(:,1);flipud(yint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
title('GPR Fit of Noise-Free Observations')
legend({'Noise-free observations','g(x) = x*sin(x)','GPR predictions','95% prediction intervals'},'Location','best')

nexttile
hold on
scatter(x_observed,y_observed2,'xr') % Observed data points
fplot(@(x) x.*sin(x),[0,10],'--r')   % Function plot of x*sin(x)
plot(x,ypred2,'g')                   % GPR predictions
patch([x;flipud(x)],[yint2(:,1);flipud(yint2(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
hold off
title('GPR Fit of Noisy Observations')
legend({'Noisy observations','g(x) = x*sin(x)','GPR predictions','95% prediction intervals'},'Location','best');