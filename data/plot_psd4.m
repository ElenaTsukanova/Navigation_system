function [ ] = plot_psd4( varargin )
% Description:  created by DAS 00249041(45941).
% Task:         to calculate and plot power spectral density function of input
% random process using periodogram method
% Changes:      1)change method periodogram - pwelch
%               2)reduce length by 256 factor
%               3)upper y limit: actual value; lower y limit: actual value
%
%               4) 26.09.2015 add fs as input argument
nVarargs = length(varargin); %%number of input arguments
if length(varargin(end)) == 1
    fs = varargin{end};
    nVarargs = nVarargs - 1;
else
    fs = 256e6;
end
    
nfft = length(varargin{1});
nfftRed = floor(nfft); %%actual number of fft points

psdVector = zeros(nVarargs, nfftRed); %%create matrix wich has on it's rows
%output psdVector of input data

%Create colour matrix for graphs
colourMatrix(1) = 'b';
colourMatrix(2) = 'r';
colourMatrix(3) = 'g';
colourMatrix(4) = 'm';
colourMatrix(5) = 'k';

figure; 

for m = 1 : nVarargs
% [psdVector, fn] = periodogram(inputVector, rectwin(nfft), nfft, 'psd', 'centered' );
[psdVector(m, : ), fn] = pwelch(varargin{m}, [], [], nfftRed, fs, 'centered');
plot(fn, 10*log10(psdVector(m, : )/1e-6), colourMatrix(m));
hold on;
end
xlabel('Frequency, Hz');
ylabel('PSD,dB/Hz');

title('Power spectral density');
yMax = max(max(10*log10(psdVector/1e-6)));
yMin = min(min(10*log10(psdVector/1e-6)));
axis([-fs/2 fs/2 yMin yMax]);
% text(-1,-1/3,'\it{xyz}');
grid on;


