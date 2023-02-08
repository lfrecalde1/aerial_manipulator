function x_filtrada = Filtro(x, t)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes 
x_filtrada(1) = x(1);
for k = 1:length(t)-1

    values = abs(x(k+1)-x(k));
    if (values > 2)
        x_filtrada(k+1) = x(k+2);
    else
        x_filtrada(k+1) = x(k+1);
    end
end


end