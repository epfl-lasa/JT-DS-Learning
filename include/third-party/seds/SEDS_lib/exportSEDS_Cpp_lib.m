function out = exportSEDS_Cpp_lib(Priors,Mu,Sigma, Data, index, x_target)
dlmwrite('Priors.txt', transpose(Priors),'newline','pc','-append','Delimiter','\t','precision','%.3f');
%dlmwrite(name, reshape(Mu/1000,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite('MU.txt', Mu,'newline','pc','-append','Delimiter','\t','precision','%.3f');
for i=1:size(Mu,2)
    %dlmwrite(name, reshape(Sigma(:,:,i)/1000^2,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
    dlmwrite('Sigma.txt', Sigma(:,:,i),'newline','pc','-append','Delimiter','\t','precision','%.3f');
end

x_init    = [];
for l = 1:length(index-1)
    x_init   = [x_init Data(1:6,l)]    
end

dlmwrite('x_init.txt', x_init,'newline','pc','-append','Delimiter','\t','precision','%.3f');
dlmwrite('x_target.txt', x_target,'newline','pc','-append','Delimiter','\t','precision','%.3f');

out = true;