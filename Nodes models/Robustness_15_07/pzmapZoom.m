function pzmapZoom(sys,zoomWindow)

subplot(1,2,1)
pzmap(sys,'k')
subplot(1,2,2)
pzmap(sys,'k')
axis(zoomWindow)
