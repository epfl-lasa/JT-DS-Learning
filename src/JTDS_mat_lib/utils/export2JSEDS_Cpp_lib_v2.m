function out = export2JSEDS_Cpp_lib_v2(Priors, Mu, Sigma, As, M)
% Export the parameters of a JT-DS model to a given file.
% 
% Arguments:
% filename - the filename to write the parameters to
% Priors - the GMM priors
% Mu - the GMM means of the JT-DS model
% Sigma - the GMM variances of the JT-DS model
% robot - the Corke robot-toolkit robot which the JT-DS model was built for
% As - the joint augmentation matrices in the JT-DS model
% M - the PCA matrix for embedding the joint-space positions into a
%   lower-dimensional latent space in the JT-DS model

% Outputs:
% out - whether the file was successfully fully written

% The parameters are written to the file in the following order:
% dimq, dimz, K
% priors
% Mus (reshaped horizontally)
% Sigmas (one matrix per each line, reshaped horizontally)
% As (one matrix per each line, reshaped horizontally)
% M (the PCA matrix) reshaped horizontally

dlmwrite('Priors.txt',Priors,'newline','pc','-append','Delimiter','\t','precision','%.6f');
dlmwrite('Mu.txt', Mu,'newline','pc','-append','Delimiter','\t','precision','%.6f');
for i=1:size(Mu,2)
    dlmwrite('Sigma.txt',Sigma(:,:,i),'newline','pc','-append','Delimiter','\t','precision','%.6f');
end
for i=1:size(Mu,2)
    dlmwrite('A_matrix.txt',As(:,:,i),'newline','pc','-append','Delimiter',' ','precision','%.6f');
end
dlmwrite('M_matrix.txt', M,'newline','pc','-append','Delimiter','\t','precision','%.6f');

out = true;
