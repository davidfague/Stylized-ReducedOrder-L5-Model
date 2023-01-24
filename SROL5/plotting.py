def get_probe(probe):
  name=str(probe.sectionname)+str(probe.sectionindex)
  segment=probe.seg
  color=probe.color
  excsecspikes=probe.excsecspikes
  inhsecspikes=probe.inhsecspikes
  return name,segment,color,excsecspikes,inhsecspikes

def plot_spikes(spikes,title):
    nTrace = len(spikes)
    z = np.zeros((nTrace,t.shape[0])) # Z is a nTrace by t.shape[0] matrix of 0's
    for i in np.arange(0,z.shape[0]):
      plt.plot(spikes[i],np.ones((spikes[i].shape[0]))*i,'k.')
    plt.ylabel('node ID')
    plt.title(title)

def plot_trace(segment,color,title,data):
  plt.plot(np.arange(0,((h.tstop)+.1),.1),data[segment],color=color)
  plt.hlines(-60,0,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(-40,0,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(0,0,tstop, color = 'grey', linestyle = 'dashed')
  plt.text(2,3,'0 mV')
  plt.hlines(-10,50,70)
  plt.vlines(50,-10,0)
  plt.text(50,-15,'10 ms')
  plt.text(2,-5,'10 mV')
  plt.text(2,-37,'-40 mV')
  plt.text(2,-57,'-60 mV')
  plt.title(title)
  plt.box(False)
  plt.xticks([])
  plt.yticks([])

def plot_voltage_trace(probe,title=None,savename=None):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  if savename==None:
    savename=name+'_Voltage.png'
  if title==None:
    title=name+' Voltage'
  plt.figure(figsize=(15,4))
  plot_trace(segment,color,title,data=v_dend)
  plt.savefig('Plots/SROL5Plots/'+savename)

def plot_voltage_trace_lastsecond(probe,title=None,savename=None):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  if savename==None:
    savename=name+'_Voltage.png'
  if title==None:
    title=name+' Voltage'
  plt.figure(figsize=(15,4))
  v=v_dend[segment]
  plt.plot(np.arange(h.tstop-1000,(h.tstop+h.dt/2),h.dt),v[int((h.tstop-1000)/h.dt)::],color=color)
  plt.hlines(-60,tstop-1000,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(-40,tstop-1000,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(0,tstop-1000,tstop, color = 'grey', linestyle = 'dashed')
  plt.text(2+tstop-1000,3,'0 mV')
  plt.hlines(-10,50+tstop-1000,70+tstop-1000)
  plt.vlines(50+tstop-1000,-10,0)
  plt.text(50+tstop-1000,-15,'10 ms')
  plt.text(2+tstop-1000,-5,'10 mV')
  plt.text(2+tstop-1000,-37,'-40 mV')
  plt.text(2+tstop-1000,-57,'-60 mV')
  plt.title(title)
  plt.box(False)
  plt.xticks([])
  plt.yticks([])
  plt.savefig('Plots/SROL5Plots/'+savename)

  
def plot_voltage_trace_last500ms(probe,title=None,savename=None,color=None):
  if color==None:
    name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  else:
    name,segment,notcolor,excsecspikes,inhsecspikes=get_probe(probe)
  duration=500
  if savename==None:
    savename=name+'_Voltage.png'
  if title==None:
    title=name+' Voltage'
  plt.figure(figsize=(15,4))
  v=v_dend[segment]
  plt.plot(np.arange(h.tstop-duration,(h.tstop+h.dt/2),h.dt),v[int((h.tstop-duration)/h.dt)::],color=color)
  plt.hlines(-60,tstop-duration,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(-40,tstop-duration,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(0,tstop-duration,tstop, color = 'grey', linestyle = 'dashed')
  plt.text(2+tstop-duration,3,'0 mV')
  plt.hlines(-10,50+tstop-duration,150+tstop-duration)
  plt.vlines(50+tstop-duration,-10,0)
  plt.text(50+tstop-duration,-15,'100 ms')
  plt.text(2+tstop-duration,-5,'10 mV')
  plt.text(2+tstop-duration,-37,'-40 mV')
  plt.text(2+tstop-duration,-57,'-60 mV')
  plt.title(title)
  plt.box(False)
  plt.xticks([])
  plt.yticks([])
  plt.savefig('Plots/SROL5Plots/'+savename)

def subplot_voltagetrace_withspikes(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  savename=name+'.png'
  plt.figure(figsize=(15,12))
  plt.subplot(3,1,1)
  title=name+' Voltage Trace'
  plot_trace(segment,color,title,data=v_dend[segment])

  t = np.arange(0,tstop,0.1)# t is an array with values ranging from 0 to t_stop with increment 0.1

  plt.subplot(3,1,2)
  title=name+' Exc Spike Trains'
  plot_spikes(excsecspikes,title)

  plt.subplot(3,1,3)
  title=name+' Inh Spike Trains'
  plot_spikes(inhsecspikes,title)
  plt.xlabel('time(ms)')
  plt.savefig('Plots/SROL5Plots/'+savename)

def subplot_voltagetrace_with1spikes(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  savename=name+'.png'
  plt.figure(figsize=(15,8))
  plt.subplot(2,1,1)
  title=name+' Voltage Trace'
  plot_trace(segment,color,title,data=v_dend[segment])
  t = np.arange(0,tstop,0.1)# t is an array with values ranging from 0 to t_stop with increment 0.1

  plt.subplot(2,1,2)
  if inhsecspikes==[]:
    title=name+' Exc Spike Trains'
    plot_spikes(excsecspikes,title)
  else:
    title=name+' Inh Spike Trains'
    plot_spikes(inhsecspikes,title)
  plt.xlabel('time(ms)')
  plt.savefig('Plots/SROL5Plots/'+savename)

#plot spikes and voltage in same plot?
# def plot_voltagetrace_withspikes(probe):
#   #work in progress: plotting spikes and voltage trace in the same plot
#   plot_voltage_trace(probe)
#   nTrace = len(probe.excspikes)
#   t = np.arange(0,tstop,0.1)
#     # t is an array with values ranging from 0 to t_stop with increment 0.1
#   z = np.zeros((nTrace,t.shape[0]))
#     # Z is a nTrace by t.shape[0] matrix of 0's
#   for i in np.arange(0,z.shape[0]):
#       plt.plot(probe.excspikes[i],np.ones((probe.excspikes[i].shape[0]))*i,'k.',color='r')
#     # figure out how to make a heatmap with spikes
#     # ax = plt.scatter(AllSegXCoord, AllSegYCoord, c = np.log(ElecDist) )
#     # plt.vlines(110,300,400)
#     # plt.text(0,350,'100 um')
#     # plt.hlines(300,110,210)
#     # plt.text(110,250,'100 um')
#     # plt.xticks([])
#     # plt.yticks([])
#     # plt.title(title)
#     # cbar = plt.colorbar()
#     # cbar.ax.set_ylabel('log(elec_distance)', rotation=270)
#     # plt.box(False)


def plot_current_trace(probe,title,savename,data):
  plt.figure(figsize=(15,4))
  plt.plot(np.arange(0,((h.tstop)+.1),.1),data[probe.seg],color=probe.color)
  plt.hlines(0,0,tstop, color = 'grey', linestyle = 'dashed')
  plt.title(title)
  plt.box(False)
  plt.xticks([])
  plt.yticks([])
  plt.savefig('Plots/SROL5Plots/'+savename)

def plot_iCaLVA(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  title=name+' iCaLVA'
  savename=name+'_iCaLVa.png'
  plot_current_trace(segment,color,title,savename,data=ical_data)

def plot_iH(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  title=name+' iH'
  savename=name+'_iH.png'
  plot_current_trace(segment,color,title,savename,data=ih_data)

def plot_iNMDA(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  title=name+' iNMDA'
  savename=name+'_iNMDA.png'
  plot_current_trace(segment,color,title,savename,data=i_NMDA_bySeg)

def plot_iCaHVA(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  title=name+' iCaHVA'
  savename=name+'_iCaHVa.png'
  plot_current_trace(probe=probe,title=title,savename=savename,data=icah_data)

def plot_axial_current(probe):
  name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  axial_current=probe.axial_current
  title=name+' Axial Current'
  dend_types = axial_current[0].dend_type
  plt.figure(figsize=(12.8, 4.8))
  for i,AC in enumerate(axial_current):
      ac = AC.get_current()
      for dend_type in dend_types:
          plt.plot(t,ac[dend_type].ravel(),label=dend_type)
      plt.ylabel('nA')
      plt.legend()
      plt.title(title)
      plt.xlabel('time (ms)')
  plt.show()

def overlay_axial_current(probes):
  plt.figure(figsize=(12.8, 4.8))
  for probe in probes:
    name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
    axial_current=probe.axial_current
    title=name+' Axial Current'
    dend_types = axial_current[0].dend_type
    plt.figure(figsize=(12.8, 4.8))
    for i,AC in enumerate(axial_current):
        ac = AC.get_current()
        for dend_type in dend_types:
            plt.plot(t,ac[dend_type].ravel(),label=dend_type)
        plt.ylabel('nA')
        plt.legend()
        plt.title(title)
        plt.xlabel('time (ms)')
  plt.show()

def overlay_voltage_trace_last500ms(probe,title=None,savename=None,color=None):
  if color==None:
    name,segment,color,excsecspikes,inhsecspikes=get_probe(probe)
  else:
    name,segment,notcolor,excsecspikes,inhsecspikes=get_probe(probe)
  duration=500
  if savename==None:
    savename=name+'_Voltage.png'
  if title==None:
    title=name+' Voltage'
  v=v_dend[segment]
  plt.plot(np.arange(h.tstop-duration,(h.tstop+h.dt/2),h.dt),v[int((h.tstop-duration)/h.dt)::],color=color)
  plt.hlines(-60,tstop-duration,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(-40,tstop-duration,tstop, color = 'grey', linestyle = 'dashed')
  plt.hlines(0,tstop-duration,tstop, color = 'grey', linestyle = 'dashed')
  plt.text(2+tstop-duration,3,'0 mV')
  plt.hlines(-10,50+tstop-duration,150+tstop-duration)
  plt.vlines(50+tstop-duration,-10,0)
  plt.text(50+tstop-duration,-15,'100 ms')
  plt.text(2+tstop-duration,-5,'10 mV')
  plt.text(2+tstop-duration,-37,'-40 mV')
  plt.text(2+tstop-duration,-57,'-60 mV')
  plt.title(title)
  plt.box(False)
  plt.xticks([])
  plt.yticks([])
  plt.savefig('Plots/SROL5Plots/'+savename)
