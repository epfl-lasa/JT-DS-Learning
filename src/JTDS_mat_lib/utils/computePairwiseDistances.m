function [D, mean_D, max_D] = computePairwiseDistances(X, display_hist)

maxSamples = 10000;
if length(X) < maxSamples
    X_train = X;
    hist_distances = 1;
else
    X_train = X(1:maxSamples, :);
    hist_distances = 10;
end

%%%%% Compute Element-wise pairwise euclidean distances %%%%%%
%%% Throughout ALL training points %%%
tic;
D = pdist(X_train, 'euclidean');
toc;
mean_D = mean(D(:));
max_D  = max(D(:));

% Visualize pairwise distances as Histogram
if (display_hist == 1)
    figure('Color',[1 1 1])
        
    D_v = D(:);           
    H = histfit(D_v(1:hist_distances:end,:));
    title('Demonstrations Pairwise Distances', 'Interpreter','LaTex')
    xlabel('$L_2$ Norm', 'Interpreter','LaTex')
    grid on
    axis tight
end

end