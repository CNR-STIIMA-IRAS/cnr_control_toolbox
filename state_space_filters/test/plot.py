#!/usr/bin/python3

import pandas as pd
import plotly.express as px

t1 = pd.read_csv('/home/feymann/.ros/test1.plt')
t3 = pd.read_csv('/home/feymann/.ros/test3.plt')
tX = pd.read_csv('/home/feymann/.ros/testX.plt')

# ofile << u << ", "  << lpf.getState() << ", " << lpf.getInput() << ","  << (lpf.getOutput()-u) << std::endl;
figX = px.line(tX, y = ['u1','u2','u3','y1','y2','y3'], title='Test Filters (testX) ')
#fig1 = px.line(t1, x = 'samples', y = 'x')
#fig1 = px.line(t1, x = 'samples', y = 'input')
#fig1 = px.line(t1, x = 'samples', y = 'y')
#fig1 = px.line(t1, x = 'samples', y = 'y-u')
figX.show()

fig3 = px.line(t3, y = ['u1','u2','u3','y1','y2','y3'], title='Test Filters (test3) ')
fig3.show()


fig1 = px.line(t1, y = ['u','x','y','(u-y).norm','(u-i).norm'], title='Test Filters (test1) ')
fig1.show()