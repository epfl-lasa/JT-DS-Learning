function out = exportSEDS_Cpp_lib(Priors,Mu,Sigma, Data, index, x_target)
dlmwrite('Priors.txt', transpose(Priors),'newline','pc','-append','Delimiter','\t','precision','%.3f');
%dlmwrite(name, reshape(Mu/1000,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
dlmwrite('MU.txt', Mu,'newline','pc','-append','Delimiter','\t','precision','%.3f');
for i=1:size(Mu,2)
    %dlmwrite(name, reshape(Sigma(:,:,i)/1000^2,1,[]),'newline','pc','-append','Delimiter',' ','precision','%.6f');
    dlmwrite('Sigma.txt', Sigma(:,:,i),'newline','pc','-append','Delimiter','\t','precision','%.3f');
end

if index(end) > length(Data)
    num_demos = length(index)-1;
else
    num_demos = length(index);
end
x_init    = zeros(6, num_demos);

for l = 1:num_demos
    x_init(:,l) = Data(1:6,index(l)); 
end

index
x_init
x_target

dlmwrite('x_init.txt', x_init,'newline','pc','-append','Delimiter','\t','precision','%.3f');
dlmwrite('x_target.txt', x_target,'newline','pc','-append','Delimiter','\t','precision','%.3f');

out = true;