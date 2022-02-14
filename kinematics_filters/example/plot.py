#!/usr/bin/python3

import pandas as pd
import plotly.express as px

pd.options.plotting.backend = "plotly"

df = pd.read_csv('test_kinematics_filter.plt')
df_melt = df.melt(id_vars='t', value_vars=['qt1', 'q1'])
df_melt2 = df.melt(id_vars='t', value_vars=['qdt1', 'qd1'])

print(df_melt)

figX = px.line(df_melt, x='t', y ='value', color='variable')
#figX = px.line(t1, x='t', y ='qt2', title='Test Kinematic Filter')
#    ['qt1','qt2','qt3','qt4','qt5','qt6','q_1','q_2','q_3','q_4','q_5','q_6',
#    ], title='Test Kinematic Filter')
figX.show()

fig2 = px.line(df_melt2, x='t', y ='value', color='variable')
#figX = px.line(t1, x='t', y ='qt2', title='Test Kinematic Filter')
#    ['qt1','qt2','qt3','qt4','qt5','qt6','q_1','q_2','q_3','q_4','q_5','q_6',
#    ], title='Test Kinematic Filter')
fig2.show()
