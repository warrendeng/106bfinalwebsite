x = linspace(0,1,150);

n = 15;
B = 8;

plot(x,B*(exp(x)-exp(-x)) + n * exp(-x));