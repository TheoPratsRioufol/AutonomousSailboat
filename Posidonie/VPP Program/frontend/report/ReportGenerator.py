from pylatex import Document, Section, Subsection, Figure, Command, Table, Tabular, Package, MultiColumn
from pylatex.utils import italic, NoEscape, bold
from PIL import Image, EpsImagePlugin
import datetime
import matplotlib.pyplot as plt

import sys
import pathlib
_parentdir = pathlib.Path(__file__).parent.parent.resolve()
_2parentdir = pathlib.Path(__file__).parent.parent.parent.resolve()
sys.path.insert(0, str(_parentdir))
sys.path.insert(0, str(_2parentdir))

from utils.Utils import *
from config.Config import *

from frontend.report.STLRenderer import *

EpsImagePlugin.gs_windows_binary = PATH_GHOSTSCRIPT

class ReportGenerator():

    """
    This class generate a PDF report of the boat sizing, with all the data required
    For the other simulator

    For ANSYS FLUENT (Hull drag analysis):
        - Position of the water line
        - Pitch, roll at equilibrium (including wind = 0)
        - Speed (and assiciated roll and pitchs) to consider

    For ANSYS AQWA (Response to wave):
        - Position of the water line
        - Pitch, roll at equilibrium (including wind = 0)
        - Inertia matrix
        - Wave to consider (wave heigh and frequency. Froode number?)
        - Speed to consider
    """

    def __init__(self, app):
        self.app = app
        self.lastPDFPath = PATH_REPORT

    def addHullImg(self, doc):
        """
        Add the image of the hull
        """
        # Create the image
        # First, put the boat in the good orientation
        self.app.getHullBuoyencyEditor().exportSTLPositionned()
        STLPath = PATH_EXPORT_STL
        img = getImgFromSTL(STLPath, color=COLOR_BOX_BORDER)
        imgPath = PATH_REPORT_IMGFOLDER + FOLDER_SEP_CHR + "hull.png"
        img.save(imgPath)

        # Add it to the report
        with doc.create(Figure(position="h!")) as fig:
            fig.add_image(imgPath, width="300px")
            fig.add_caption(NoEscape(r"The considered hull, located at: \protect\path{"+STLPath.replace('\\','/')+"}"))

    def getInertiaMatrix(self):
        """
        Return the latex code of the inertia matrix
        """
        # Update the boat inertia
        self.app.getBoat().getSolver().computeBoatInertia()
        inertia = self.app.getBoat().getSolver().getBoatInertia()
        inertia = inertia.getMatrix().valueIn(Base.BOAT)
        # Transform it to a table
        I = [["{:.2f}".format(inertia[i,j]) for j in range(len(inertia[i]))] for i in range(len(inertia))]
        
        latex = r"""$I_{boat}=\begin{bmatrix}
        """
        for i in I:
            for j in i:
                latex += j + " & "
            latex = latex[:-2] + r"""\\
                """
        latex += r"\end{bmatrix}_{O,B_b} kg.m2$"
        return latex

    def addSizing(self, doc):
        """
        Add the sizing of the boat
        """
        geom = self.app.getBoat().getSolver().getGlobalGeom()
        with doc.create(Tabular('|cccp{7cm}|')) as table:
            table.add_hline()
            for comp in geom:
                # add comp name
                table.add_row((MultiColumn(4, align="|l|", data=NoEscape(r"\cellcolor[gray]{.9} \textsc{"+comp.upper() + "}")),))
                for elm in geom[comp]:
                    table.add_row((NoEscape(r"\texttt{"+elm+"}"), geom[comp][elm]['value'], geom[comp][elm]['unit'].value, geom[comp][elm]['info']))
                table.add_hline()

        # Then add an image of the complete boat
        bv = self.app.getBoatViewver()
        
        # First, save the initial configuration
        originalDisplayPolicy = bv.getDisplayPolicy()
        originalFilledPolygons = bv.getFilledPolygons()

        # Then, aplly the style of the report
        bv.resetCamera()
        bv.setFilledPolygons(True)
        bv.setDisplayPolicyFromDic(REPORT_DISPLAY_POLICY)
        bv.fitCamera()

        # Update the boat viewver
        bv.displayFrame()
        bv.displayFrame()

        # Export and save an image
        canvas = bv.getCanvas()
        canvas.update()
        canvas.postscript(file=PATH_REPORT_IMGFOLDER + 'boat.ps') 
        img = Image.open(PATH_REPORT_IMGFOLDER + 'boat.ps') 
        img.save(PATH_REPORT_IMGFOLDER + 'boat.png', 'png') 

        # Restore the initial style
        bv.setFilledPolygons(originalFilledPolygons)
        bv.setDisplayPolicyFromDic(originalDisplayPolicy)
        
        # Add it to the report
        with doc.create(Figure(position="h!")) as fig:
            fig.add_image(PATH_REPORT_IMGFOLDER + 'boat.png', width="300px")
            fig.add_caption(r"The assembled boat")


    def addStaticData(self, doc):
        """
        Add Static information about the boat
        """
        with doc.create(Subsection("Données sur le bateau")):
            solver = self.app.getBoat().getSolver()
            hbc = solver.getHull().getHullBuoyencyCalculator()
            hbe = self.app.getHullBuoyencyEditor()
                
            doc.append('Donnée sur le dimentionement:')
            doc.append(NoEscape("""
\medskip
            """))
            #doc.append(Command('centering'))
            with doc.create(Tabular('|p{6cm}c|')) as table:
                table.add_hline()
                # Dimmention
                table.add_row(("Longeur de la coque", NoEscape(r"\textbf{"+"{:.2f} m".format(solver.getHull().getGeomP('lh'))+"}")))
                table.add_row(("Largeur de la coque", "{:.2f} m".format(solver.getHull().getGeomP('whmid'))))
                table.add_row(("Hauteur de la coque", "{:.2f} m".format(solver.getHull().getGeomP('hhmid'))))
                htote = solver.getHull().getGeomP('hhmid') + solver.getSail().getGeomP('ds') + solver.getSail().getGeomP('hms') + 2*solver.getSail().getGeomP('he')
                htot = htote + solver.getDrift().getGeomP('hd')
                table.add_row(("Hauteur du bateau", NoEscape(r"\textbf{"+"{:.2f} m".format(htot)+"}")))
                table.add_row(("Hauteur du bateau, émergé", NoEscape(r"\textbf{"+"{:.2f} m".format(htote)+"}")))
                table.add_hline()

            doc.append(NoEscape("""
\medskip
Donnée sur la masse:
                                
\medskip                  
            """))
            #doc.append(Command('centering'))
            with doc.create(Tabular('|p{6cm}c|')) as table:
                # Mass
                table.add_hline()
                table.add_row(("Masse de la voile", "{:.2f} kg".format(solver.getSail().getWeight())))
                table.add_row(("Masse de la dérive (Plaque)", "{:.2f} kg".format(solver.getDrift().getPlateMass())))
                table.add_row(("Masse de la dérive (Lest)", "{:.2f} kg".format(solver.getDrift().getGeomP('mLest'))))
                table.add_row(("Masse du safran", "{:.2f} kg".format(solver.getRudder().getWeight())))
                table.add_row(("Masse de la coque", "{:.2f} kg".format(solver.getHull().getWeight())))
                table.add_row(("Masse totale", NoEscape(r"\textbf{"+"{:.2f} kg".format(solver.getBoatMass())+"}")))
                table.add_hline()
            
            doc.append(NoEscape("""
\medskip
Donnée sur les volumes:

\medskip      
            """))
            #doc.append(Command('centering'))
            with doc.create(Tabular('|p{6cm}c|')) as table:
                # Volumes
                table.add_hline()
                vplate   = solver.getDrift().getPlateVolume()
                vlest    = solver.getDrift().getLestVolume()
                vrudder  = solver.getRudder().getRudderVolume()
                vprudder = solver.getRudder().getProtectRudderVolume()
                vcomp = vplate+vlest+vrudder+vprudder
                table.add_row(("Volume de la dérive (Plaque)", "{:.2f} L".format(vplate*1e3)))
                table.add_row(("Volume de la dérive (Lest)", "{:.2f} L".format(vlest*1e3)))
                table.add_row(("Volume du safran (safran)", "{:.2f} L".format(vrudder*1e3)))
                table.add_row(("Volume du safran (protège)", "{:.2f} L".format(vprudder*1e3)))
                table.add_row(("Volume hors coque", NoEscape(r"\textbf{"+"{:.2f} L".format(vcomp*1e3)+"}")))
                table.add_row(("Volume de la coque", NoEscape(r"\textbf{"+"{:.2f} L".format(hbc.getImergedVolume()*1e3)+"}")))
                table.add_hline()
            
            doc.append(NoEscape("""
\medskip
Donnée sur les paramètres mécaniques (dans $R_b$ par défaut). O désigne l'origine du repère $R_b$:

\medskip           
            """))
            #doc.append(Command('centering'))
            with doc.create(Tabular('|p{6cm}c|')) as table:
                # Inertia
                table.add_hline()
                table.add_row((NoEscape(r"Inertie du bateau"), NoEscape(self.getInertiaMatrix())))
                # CdC and Cdg
                table.add_row((NoEscape(r"Position du CdG dans le fichier STL"),"({:.3f}, {:.2f}, {:.2f})".format(*hbe.getCdg())))
                table.add_row(("Composante z du centre de gravité (hors coque)", "{:.2f} m".format(solver.computeAndGetComponentsCdg().valueIn(Referential.BOAT)[2])))
                table.add_row(("Composante z du centre de gravité", "{:.2f} m".format(solver.getBoatCdg().valueIn(Referential.BOAT)[2])))
                table.add_row(("Composante z du centre de rotation", "{:.2f} m".format(solver.getBoatOrigin().valueIn(Referential.BOAT)[2])))
                table.add_hline()

            doc.append(NoEscape("""
\medskip
Donnée à l'équilibre du bateau, sans vent et sans vitesse. (Note: Dans $R_b$, le niveau de l'eau se situe en z=0)

\medskip          
            """))
            #doc.append(Command('centering'))
            with doc.create(Tabular('|p{6cm}c|')) as table:
                # Equilibrium
                table.add_hline()
                ZplaneInSTL = hbc.coordinateToSTL([0, 0, -float(hbe.getEqZ())])
                posix = ""
                if not self.app.getHullBuoyencyEditor().getEqCalculusState():
                    posix = "(NON A JOUR)"
                table.add_row((NoEscape(r"Position $z_0$ du bateau dans $R_b$"), "{:.3f}".format(float(hbe.getEqZ())) + " m" + posix))
                table.add_row((NoEscape(r"Tanguage $\psi_0$ du bateau dans $R_b$"), "{:.2f}".format(float(hbe.getEqPitch())) + " deg" + posix))
                table.add_row((NoEscape(r"Position $z_0$ du bateau dans le référentiel du fichier STL"),"({:.3f}, {:.2f}, {:.2f})".format(*ZplaneInSTL) + posix))
                
                table.add_hline()


        # Add the stability curve
        with doc.create(Subsection("Analyse de stabilité")):
            doc.append("L'analyse de stabilité est effectué à l'équilibre vertical, entre la masse du bateau et les forces de flottaison.")

            # Draw the figure - DISPLACEMENT
            fig = plt.figure(figsize=(8, 5), dpi=300)
            ax = fig.add_subplot(1,1,1)
            ax2 = ax.twinx()
            self.app.getStabilityEditor().drawDisplacementCurve(ax, ax2)
            ax.set_title('')

            # Save the figure
            displacementPath = PATH_REPORT_IMGFOLDER + 'stability-displacement.png'
            fig.tight_layout()
            fig.savefig(displacementPath)
            
            # Draw the figure - ROLL STABILITY
            fig = plt.figure(figsize=(8, 5), dpi=300)
            ax = fig.add_subplot(1,1,1)
            self.app.getStabilityEditor().drawStabilityCurve(ax)
            ax.set_title('')

            # Save the figure
            rollPath = PATH_REPORT_IMGFOLDER + 'stability-roll.png'
            fig.tight_layout()
            fig.savefig(rollPath)

            # Draw the figure - PITCH STABILITY
            fig = plt.figure(figsize=(8, 5), dpi=300)
            ax = fig.add_subplot(1,1,1)
            self.app.getStabilityEditor().drawStabilityCurve(ax, rollAxis=False)
            ax.set_title('')

            # Save the figure
            pitchPath = PATH_REPORT_IMGFOLDER + 'stability-pitch.png'
            fig.tight_layout()
            fig.savefig(pitchPath)

            # Then add the figure to the report
            with doc.create(Figure(position="h!")) as figure:
                figure.add_image(displacementPath, width="400px")
                figure.add_caption(NoEscape(r"Courbe de déplacement du bateau en \textbf{altitude}"))

            with doc.create(Figure(position="h!")) as figure:
                figure.add_image(rollPath, width="400px")
                figure.add_caption(NoEscape(r"Courbe de stabilité du bateau en \textbf{rouli}, pour différent tanguage"))

            with doc.create(Figure(position="h!")) as figure:
                figure.add_image(pitchPath, width="400px")
                figure.add_caption(NoEscape(r"Courbe de stabilité du bateau en \textbf{tanguage}, pour différent roulis"))


    def addDynamicData(self, doc):
        """
        Add the dynamic data to the report
        """
        polar = self.app.getPolarGenerator()

        # Recreate the polar plot in a good format for the report
        fig = plt.figure(figsize=(8, 4), dpi=300)
        axS = fig.add_subplot(1,2,1, projection='polar')
        axR = fig.add_subplot(1,2,2, projection='polar')

        n = 0
        for trace in polar.getTraces():
            n += 1
            color = getPlotColor(n)
            speeds = ms2noeud(np.array(trace.speeds))
            rolls = np.rad2deg(np.array(trace.rolls))
            angs = np.array(trace.angs)

            # Plot speed
            axS.plot(angs, speeds, color+'.-', label=trace.getLabel())
            axS.plot(-angs, speeds, color+'.-')

            # Plot roll
            axR.plot(angs, rolls, color+'.-', label=trace.getLabel())
            axR.plot(-angs, rolls, color+'.-')

        # Add the legend
        axS.legend(loc='lower center', bbox_to_anchor=(0.5, -0.3))
        axR.legend(loc='lower center', bbox_to_anchor=(0.5, -0.3))
        axS.set_title('Vitesse')
        axR.set_title('Roulis')

        # Save the polar figure
        figPath = PATH_REPORT_IMGFOLDER + 'polar.png'
        fig.tight_layout()
        fig.savefig(figPath)

        # Then add the global polar plot figure to the report
        with doc.create(Figure(position="h!")) as figure:
            figure.add_image(figPath, width="400px")
            figure.add_caption("Polaire de vitesse du bateau")

        for trace in polar.getTraces():
            # Create a tabular of the result
            with doc.create(Table(position="h!")) as tableFig:
                doc.append(Command('centering'))
                with doc.create(Tabular('|c|c|c|c|')) as table:
                    table.add_hline()
                    table.add_row(("Angle du vent réel [deg]", "Vitesse [nds]", "Gîte [deg]", 'Tanguage [deg]'))
                    
                    # Save the maximum to plot them in bold
                    extremums = {}
                    if (len(trace.speeds) > 0):
                        extremums['speed'] = {'min':min(trace.speeds), 'max':max(trace.speeds)}
                        extremums['roll'] = {'min':min(trace.rolls), 'max':max(trace.rolls)}
                        extremums['pitch'] = {'min':min(trace.pitchs), 'max':max(trace.pitchs)}
                    
                    for i in range(len(trace.angs)):
                        
                        cell = {}
                        cell['speed'] = {'val':trace.speeds[i], 'str': "{:.2f}".format(ms2noeud(trace.speeds[i]))}
                        cell['roll'] = {'val':trace.rolls[i], 'str': "{:.2f}".format(np.rad2deg(trace.rolls[i]))}
                        cell['pitch'] = {'val':trace.pitchs[i], 'str': "{:.2f}".format(np.rad2deg(trace.pitchs[i]))}
                        
                        for dof in cell:
                            if (cell[dof]['val'] in [extremums[dof]['min'], extremums[dof]['max']]):
                                cell[dof]['str'] = r"\textbf{" + cell[dof]['str'] + "}"

                        table.add_row(("{:.2f}".format(np.rad2deg(trace.angs[i])),
                                    NoEscape(cell['speed']['str']),
                                    NoEscape(cell['roll']['str']),
                                    NoEscape(cell['pitch']['str'])))
                    table.add_hline()

                # Get the setup
                setup = trace.getSetup()
                setup = r'Cette courbe a été obtenue en utilisant la stratégie \textbf{'+setup['strategy']+r"}, Avec \textbf{"+setup['nbHeading']+r"} points d'échantillonage. Dans le cas de la stratégie par \texttt{reset}, le régime permanent est obtenu pour une variation angulaire inférieure à \textbf{"+setup['steadyAng']+r" deg}, pendant une durée supérieure à \textbf{"+setup['steadyTime']+r" s}. Une erreur maximale de \textbf{"+setup['targetError']+r" deg} sur la consigne est tolérée"
                
                tableFig.add_caption(NoEscape(r'Résultat détaillés pour la courbe \texttt{' + trace.getLabel() + '}. ' + setup))
                

    def addMxyData(self, doc):
        """
        Add the data of the Mxy Curve
        """
        mxFigure = self.app.getMxPlot().getMxFigure()
        currentGeomStamp = self.app.getGeomStamp()

        # Recreate the polar plot in a good format for the report
        figs = []
        figs.append(plt.figure(figsize=(8, 4), dpi=300))
        ax1 = figs[-1].add_subplot(1,1,1)

        figs.append(plt.figure(figsize=(8, 4), dpi=300))
        ax2 = figs[-1].add_subplot(1,1,1)

        figs.append(plt.figure(figsize=(8, 4), dpi=300))
        ax3 = figs[-1].add_subplot(1,1,1)

        for trace in mxFigure.getTraces():
            # First, the speed plot
            ax1.plot(trace.wspeeds, ms2noeud(trace.bspeeds), label="Speed @h={:.2f}°".format(np.rad2deg(trace.heading)))
            
            # Then the angular plot
            ax2.plot(trace.wspeeds, np.rad2deg(trace.rolls), '--', label="Roll @h={:.2f}°".format(np.rad2deg(trace.heading)))
            ax2.plot(trace.wspeeds, np.rad2deg(trace.pitchs), '-', label="Pitch @h={:.2f}°".format(np.rad2deg(trace.heading)))
            
            # Finally the moments plot
            ax3.plot(trace.wspeeds, trace.mxs, '--', label="Mx @h={:.2f}°".format(np.rad2deg(trace.heading)))
            ax3.plot(trace.wspeeds, trace.mys, '-', label="Mx @h={:.2f}°".format(np.rad2deg(trace.heading)))
            
        # Add the legend
        ax1.legend()
        ax2.legend()
        ax3.legend()
        ax1.set_xlabel('Vitesse du vent [nds]')
        ax2.set_xlabel('Vitesse du vent [nds]')
        ax3.set_xlabel('Vitesse du vent [nds]')

        ax1.set_ylabel('Vitesse du bateau [nds]')
        ax2.set_ylabel('Mouvement angulaire [deg]')
        ax3.set_ylabel('Moment [Nm]')
        legends = {0:"Evolution de la vitesse du bateau en fonction de celle du vent",
                   1:"Evolution de la gite et du tanguage du bateau en fonction de celle du vent",
                   2:"Evolution des moments combinés de la voile, safran et dérive en fonction de celle du vent"}

        # Save the polar figure
        for i in range(3):
            figPath = PATH_REPORT_IMGFOLDER + f'mxycurve{i}.png'
            figs[i].tight_layout()
            figs[i].savefig(figPath)

            # Then add the global polar plot figure to the report
            with doc.create(Figure(position="h!")) as figure:
                figure.add_image(figPath, width="400px")
                figure.add_caption(legends[i])

        ultimateConditions = {}
        for trace in mxFigure.getTraces():
            # Create a tabular of the result


            # For each trace, save the ultimate wind speed
            ultimateConditions[trace.heading] = {}

            with doc.create(Table(position="h!")) as tableFig:
                doc.append(Command('centering'))
                with doc.create(Tabular('|c|c|c|c|c|c|')) as table:
                    table.add_hline()
                    table.add_row(("Vitesse du vent [nds]", "Vitesse [nds]", "Gîte [deg]", 'Tanguage [deg]', 'Mx [Nm]', 'My [Nm]'))
                        
                    # Save the maximum to plot them in bold
                    extremums = {}
                    if (len(trace.wspeeds) > 0):
                        extremums['bspeed'] = {'min':min(trace.bspeeds), 'max':max(trace.bspeeds)}
                        extremums['roll'] = {'min':min(trace.rolls), 'max':max(trace.rolls)}
                        extremums['pitch'] = {'min':min(trace.pitchs), 'max':max(trace.pitchs)}
                        extremums['mx'] = {'min':min(trace.mxs), 'max':max(trace.mxs)}
                        extremums['my'] = {'min':min(trace.mys), 'max':max(trace.mys)}
                        
                    for i in range(len(trace.wspeeds)):
                            
                        cell = {}
                        cell['bspeed'] = {'val':trace.bspeeds[i], 'str': "{:.2f}".format(ms2noeud(trace.bspeeds[i]))}
                        cell['mx'] = {'val':trace.mxs[i], 'str': "{:.2f}".format(trace.mxs[i])}
                        cell['my'] = {'val':trace.mys[i], 'str': "{:.2f}".format(trace.mys[i])}
                        cell['roll'] = {'val':trace.rolls[i], 'str': "{:.2f}".format(np.rad2deg(trace.rolls[i]))}
                        cell['pitch'] = {'val':trace.pitchs[i], 'str': "{:.2f}".format(np.rad2deg(trace.pitchs[i]))}
                            
                        for dof in cell:
                            if (cell[dof]['val'] in [extremums[dof]['min'], extremums[dof]['max']]):
                                cell[dof]['str'] = r"\textbf{" + cell[dof]['str'] + "}"

                        # add color for extream roll or pitch
                        criticalReach = False
                        
                        if (np.abs(cell['roll']['val']) > CRITICAL_ROLL):
                            cell['roll']['str'] = r"\textcolor{red}{" + cell['roll']['str'] + "}"
                            criticalReach = True

                        if (np.abs(cell['pitch']['val']) > CRITICAL_PITCH):
                            cell['pitch']['str'] = r"\textcolor{red}{" + cell['pitch']['str'] + "}"
                            criticalReach = True

                        if criticalReach and (('wspeed' not in ultimateConditions[trace.heading]) or (ultimateConditions[trace.heading]['wspeed'] > trace.wspeeds[i])):
                            ultimateConditions[trace.heading]['wspeed'] = trace.wspeeds[i]
                            ultimateConditions[trace.heading]['bspeed'] = cell['bspeed']['val']
                            ultimateConditions[trace.heading]['roll'] = cell['roll']['val']
                            ultimateConditions[trace.heading]['pitch'] = cell['pitch']['val']

                        table.add_row(("{:.2f}".format(trace.wspeeds[i]),
                                        NoEscape(cell['bspeed']['str']),
                                        NoEscape(cell['roll']['str']),
                                        NoEscape(cell['pitch']['str']),
                                        NoEscape(cell['mx']['str']),
                                        NoEscape(cell['my']['str'])))
                    table.add_hline()

                    posix = ""
                    if (trace.geomStamp != currentGeomStamp):
                        posix = r" \textbf{NON A JOUR}"
                    else:
                        posix = r" (\textit{A JOUR})"
                    tableFig.add_caption(NoEscape("Résultats détaillé pour l'analyse de stabilité dynamique en fonction du vent. Résultats pour un cap de {:.2f}°.".format(np.rad2deg(trace.heading))+posix))

            if (ultimateConditions[trace.heading] == {}):
                ultimateConditions[trace.heading]['wspeed'] = trace.wspeeds[-1]
                ultimateConditions[trace.heading]['roll'] = trace.rolls[-1]
                ultimateConditions[trace.heading]['pitch'] = trace.pitchs[-1]
                ultimateConditions[trace.heading]['bspeed'] = trace.bspeeds[-1]

        
        print("ultimateConditions=",ultimateConditions)
        # Finally, create the ultimate condition table and figure
        outdated = False
        with doc.create(Table(position="h!")) as tableFig:
            doc.append(Command('centering'))
            with doc.create(Tabular('|c|c|c|c|c|')) as table:
                table.add_hline()
                table.add_row(("Cap [deg]","Vitesse de vent critique [nds]","Vitesse du bateau [nds]","Gîte [deg]","Tangage [deg]"))
                table.add_hline()
                table.add_hline()
                for trace in mxFigure.getTraces():
                    if trace.geomStamp != currentGeomStamp:
                        outdated = True
                    ucond = ultimateConditions[trace.heading]
                    table.add_row(("{:.2f}".format(np.rad2deg(trace.heading)),
                                    "{:.2f}".format(ucond['wspeed']),
                                    "{:.2f}".format(ms2noeud(ucond['bspeed'])),
                                    "{:.2f}".format(np.rad2deg(ucond['roll'])),
                                    "{:.2f}".format(np.rad2deg(ucond['pitch']))))
                table.add_hline()
            
            posix = ""
            if (outdated):
                posix = r" \textbf{NON A JOUR}"
            else:
                posix = r" (\textit{A JOUR})"
            tableFig.add_caption(NoEscape("Vitesse de vent critique pour plusieurs caps" + posix))

        # Then visualiaze the sailing domain

        fig = plt.figure(figsize=(4, 4), dpi=300)
        ax = fig.add_subplot(1,1,1, projection='polar')

        headings = np.array(sorted(list(ultimateConditions.keys())))
        criticalW = [ultimateConditions[h]['wspeed'] for h in headings]

        ax.fill_between(list(-headings) + list(reversed(list(headings))), criticalW + list(reversed(criticalW)), [0]*2*len(headings), color='red', alpha=0.5)

        figPath = PATH_REPORT_IMGFOLDER + 'fullPowerNavigationDomain.png'
        fig.tight_layout()
        fig.savefig(figPath)

        # Then add the global polar plot figure to the report
        with doc.create(Figure(position="h!")) as figure:
            figure.add_image(figPath, width="400px")
            if (outdated):
                posix = r" \textbf{NON A JOUR}"
            else:
                posix = r" (\textit{A JOUR})"
            figure.add_caption(NoEscape("Domaine de navigation à pleine puissance. Le rayon représente la vitesse du vent et l'angle est celui du vent réel." + posix))


    
    def addModelData(self, doc):
        """
        Add the model data to the report
        """
        with doc.create(Subsection("Profil Aérodynamique")):
            # Load the naca model
            naca = self.app.getNACAEditor()
            fig = plt.figure(figsize=(8, 4), dpi=200)

            for profile in naca.getProfiles():
                ax, ax2 = naca.plot(fig, profile)
                # Save the figure
                figPath = PATH_REPORT_IMGFOLDER + 'aeromodel.png'
                ax.set_title('')
                fig.tight_layout()
                fig.savefig(figPath)

                # Then add the plot to the report
                with doc.create(Figure(position="h!")) as figure:
                    figure.add_image(figPath, width="400px")
                    figure.add_caption(NoEscape(r"Modèle aérodynamique pour le profil \texttt{" + profile + '}'))

        with doc.create(Subsection("Modèle de flottaison")):
            # Load the buoyency editor
            hbe = self.app.getHullBuoyencyEditor()
            doc.append(Command('centering'))
            with doc.create(Tabular('|l|c|')) as table:
                table.add_hline()
                table.add_row((NoEscape(r"Angle maximal de Rouli $\theta$"), "{:.2f}".format(np.rad2deg(hbe.getMaxRoll()))))
                table.add_row((NoEscape(r"Angle maximal de Tanguage $\psi$"), "{:.2f}".format(np.rad2deg(hbe.getMaxPitch()))))
                table.add_row((NoEscape(r"Angle minimal de Tanguage $\psi$"), "{:.2f}".format(np.rad2deg(hbe.getMinPitch()))))
                table.add_row((NoEscape(r"Nombre de points en Roulis $\theta$:"), hbe.getNRoll()))
                table.add_row((NoEscape(r"Nombre de points en Tanguage $\psi$:"), hbe.getNPitch()))
                table.add_row((NoEscape(r"Nombre de points en $z$:"), hbe.getNZ()))
                table.add_row((NoEscape(r"Nombre total de points:"), hbe.getNZ()*hbe.getNRoll()*hbe.getNPitch()))
                table.add_hline()
                    


        
    def generate(self):
        print("[INFO] Generating report...")
        self.newPDFPath()
        
        doc = Document()
        doc.preamble.append(Package("amsmath"))
        doc.preamble.append(Package("xcolor", ("table",)))
        doc.preamble.append(Package("colortbl"))
        doc.preamble.append(Package("url"))
        doc.preamble.append(Command("title", "Dimentionement de Posidonie"))
        doc.append(NoEscape(r"\maketitle"))

        self.addHullImg(doc)
        doc.append(NoEscape("Ce document à été généré le " + datetime.datetime.now().strftime("%d %B %Y à %H:%M:%S")))

        with doc.create(Section("Géométrie")):
            self.addSizing(doc)

        with doc.create(Section("Données statiques")):
            self.addStaticData(doc)

        with doc.create(Section("Données dynamique")):
            self.addDynamicData(doc)

        with doc.create(Section("Stabilitée dynamique")):
            self.addMxyData(doc)

        with doc.create(Section("Modèles")):
            self.addModelData(doc)
        
        # Generate the PDF
        doc.generate_pdf(self.getPDFPath(), clean_tex=True, compiler="pdflatex")

        print("Report Generated")

        tk.messagebox.showinfo("Report Generator", f"The report has been successfully generated.\nLocation: {self.getPDFPath()}") 

    def getPDFPath(self):
        return self.lastPDFPath
    
    def newPDFPath(self):
        self.lastPDFPath = PATH_REPORT + datetime.datetime.now().strftime("_%d-%B-%Y_%H-%M-%S")

    def getSaveDic(self):
        """
        Return the save dic of this component
        """
        return {'lastPDFPath':self.lastPDFPath}
    
    def load(self, save):
        """
        Load a save dictionnary
        """
        self.lastPDFPath = save['lastPDFPath']