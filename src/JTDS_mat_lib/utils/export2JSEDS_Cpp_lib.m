function out = export2JSEDS_Cpp_lib(filename,Priors,Mu,Sigma,robot, As,M)
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
% Robot DH parameters: all d's, then all a's, then all alpha's, then all
% qmin's, then all qmax's

dlmwrite(filename, [size(M, 2), size(Mu)],'Delimiter',' ','precision','%i');
dlmwrite(filename, Priors,'newline','pc','-append','Delimiter',' ','precision','%.6f');
%dlmwrite(name, reshape(Mu/1000,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite(filename, reshape(Mu,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
for i=1:size(Mu,2)
    %dlmwrite(name, reshape(Sigma(:,:,i)/1000^2,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
    dlmwrite(filename, reshape(Sigma(:,:,i)',1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
end
for i=1:size(Mu,2)
    %dlmwrite(name, reshape(Sigma(:,:,i)/1000^2,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
    dlmwrite(filename, reshape(As(:,:,i)',1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
end
dlmwrite(filename, reshape(M,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
for i = 1:robot.n
    a(i) = robot.links(i).a;
    d(i) = robot.links(i).d;
    alpha(i) = robot.links(i).alpha;
    qmin(i) = robot.links(i).qlim(1);
    qmax(i) = robot.links(i).qlim(2);
end
dlmwrite(filename, d,'newline','pc','-append','Delimiter',' ','precision','%.4f');
dlmwrite(filename, a,'newline','pc','-append','Delimiter',' ','precision','%.4f');
dlmwrite(filename, alpha,'newline','pc','-append','Delimiter',' ','precision','%.4f');
dlmwrite(filename, qmin,'newline','pc','-append','Delimiter',' ','precision','%.4f');
dlmwrite(filename, qmax,'newline','pc','-append','Delimiter',' ','precision','%.4f');

out = true;
