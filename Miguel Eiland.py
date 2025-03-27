import numpy as np
import openpyxl

wb_nodes = openpyxl.load_workbook('nodes_DXB.xlsx')
wb_edges = openpyxl.load_workbook('edges_DXB.xlsx')
nodes = wb_nodes['Sheet1']
edg = wb_edges['Sheet1']

stap = 0.5
x_start = 0
x_eind = 25*stap
y_start = 0.5
y_eind = 0.5

if round(x_eind-x_start,0) != 0:
    x_nodes = list(np.arange(x_start, x_eind, stap))
    x_nodes.append(x_eind)
    y_nodes = np.zeros(len(x_nodes))+y_start
else:
    y_nodes = list(np.arange(y_start, y_eind, stap))
    y_nodes.append(y_eind)
    x_nodes = np.zeros(len(y_nodes))+x_start

free_nodes = False
free_edges = False
first_free = 1
second_free = 1
while not free_nodes:
    first_free += 1
    if nodes['A'+str(first_free)].value == None:
        free_nodes = True
while not free_edges:
    second_free += 1
    if edg['A'+str(second_free)].value == None:
        free_edges = True

for i in range(len(x_nodes)):
    nodes['A' + str(first_free)] = first_free
    nodes['B' + str(first_free)] = x_nodes[i]
    nodes['C' + str(first_free)] = y_nodes[i]
    nodes['D' + str(first_free)] = 'between'

    if i != len(x_nodes)-1:
        edg['A'+str(second_free)] = first_free
        edg['B'+str(second_free)] = first_free+1
        edg['C'+str(second_free)] = stap
        edg['A'+str(second_free+1)] = first_free+1
        edg['B'+str(second_free+1)] = first_free
        edg['C'+str(second_free+1)] = stap

        first_free += 1
        second_free += 2

wb_nodes.save('nodes_DXB.xlsx')
wb_edges.save('edges_DXB.xlsx')