% Input a set of numbers as an n-dimension array, get their least common multiple.
function output = lcms(numberArray)
    numberArray = reshape(numberArray, 1, []);
    for i = 1:size(numberArray,2)
        temp = factor(numberArray(i));
        for j = 1:size(temp,2)
            output(i,j) = temp(1,j); %#ok<*AGROW>
        end
    end
    p = primes(max(max(output)));
    q = zeros(size(p));
    for i = 1:size(p,2)
        for j = 1:size(output,1)
            temp = length(find(output(j,:) == p(i)));
            if(temp > q(1,i))
                q(1,i) = temp;
            end
        end
    end
    z = p.^q;
    output = 1;
    for i = 1:size(z,2)
        output = output*z(1,i);
    end
end