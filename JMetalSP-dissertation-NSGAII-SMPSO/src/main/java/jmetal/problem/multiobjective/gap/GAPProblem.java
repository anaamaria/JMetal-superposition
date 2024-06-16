package jmetal.problem.multiobjective.gap;
import jmetal.core.problem.doubleproblem.impl.AbstractDoubleProblem;
import jmetal.core.solution.doublesolution.DoubleSolution;
import jmetal.core.util.errorchecking.JMetalException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

public class GAPProblem extends AbstractDoubleProblem {

    private final String outputFile = "C:\\Users\\Ana\\Desktop\\FACULTATE\\Dissertation\\output.xml";
    private static final String configFile = "C:\\Users\\Ana\\Desktop\\FACULTATE\\Dissertation\\configs\\designSpace\\config.xml";

    DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
    private int counter = 1;

    public GAPProblem() throws ParserConfigurationException, IOException, org.xml.sax.SAXException {
        this(6, 2);
    }

    public GAPProblem(Integer numberOfVariables, Integer numberOfObjectives) throws JMetalException, ParserConfigurationException, IOException, org.xml.sax.SAXException {
        numberOfObjectives(numberOfObjectives);
        name("GAP");

        List<Double> lowerLimit = new ArrayList<>(numberOfVariables);
        List<Double> upperLimit = new ArrayList<>(numberOfVariables);

        IntStream.range(0, numberOfVariables).forEach(i -> {
            lowerLimit.add(1.0);
        });
        upperLimit.add(16.0);
        upperLimit.add(512.0);
        upperLimit.add(512.0);
        upperLimit.add(16.0); // Upper limit for c_chunk
        upperLimit.add(8192.0); // Upper limit for c_sets
        upperLimit.add(128.0); // Upper limit for c_lines

        variableBounds(lowerLimit, upperLimit);
    }

    @Override
    public DoubleSolution evaluate(DoubleSolution solution) {
        double[] result;
        System.out.println();
        System.out.println();
        System.out.println("Iteration = " + counter);
        counter++;
        int nLines = (int) Math.round(solution.variables().get(0));
        int nColumns = (int) Math.round(solution.variables().get(1));
        int nLayers = (int) Math.round(solution.variables().get(2));
        int cChunk = (int) Math.round(solution.variables().get(3));
        int cSets = (int) Math.round(solution.variables().get(4));
        int cLines = (int) Math.round(solution.variables().get(5));

        try {
            changeParameters(dbf, configFile, configFile, nLines, nColumns, nLayers, cChunk, cSets, cLines);
            displayXmlParameters(dbf, configFile);
            System.out.println();
            System.out.println("Running GAPSimulator");
            callSimulator(configFile, outputFile);
            result = parseOutputXml(dbf, outputFile);
            displayOutputXml(dbf, outputFile);
        } catch (ParserConfigurationException | IOException | SAXException | TransformerException e) {
            throw new RuntimeException(e);
        }

        solution.objectives()[0] = 0 - result[0];
        solution.objectives()[1] = result[1];

        return solution;
    }

    private static double[] parseOutputXml(DocumentBuilderFactory dbf, String fileName) throws ParserConfigurationException, IOException, org.xml.sax.SAXException {
        DocumentBuilder db = dbf.newDocumentBuilder();
        Document doc = db.parse(new File(fileName));
        doc.getDocumentElement().normalize();

        NodeList list = doc.getElementsByTagName("result");
        for (int temp = 0; temp < list.getLength(); temp++) {
            Node node = list.item(temp);
            if (node.getNodeType() == Node.ELEMENT_NODE) {
                Element element = (Element) node;
                String ipc = element.getElementsByTagName("ipc").item(0).getTextContent();
                String energy = element.getElementsByTagName("energy").item(0).getTextContent();
                double[] returnValue = new double[]{Double.parseDouble(ipc), Double.parseDouble(energy)};
                return returnValue;
            }
        }
        return null;
    }

    private static void changeParameters(DocumentBuilderFactory dbf, String fileName, String modifiedFile, int nLines, int nColumns, int nLayers, int cChunk, int cSets, int cLines) throws ParserConfigurationException, IOException, org.xml.sax.SAXException, TransformerException {
        try {
            DocumentBuilder db = dbf.newDocumentBuilder();
            Document doc = db.parse(new File(fileName));
            doc.getDocumentElement().normalize();

            NodeList list = doc.getElementsByTagName("parameter");
            for (int temp = 0; temp < list.getLength(); temp++) {
                Node node = list.item(temp);
                if (node.getNodeType() == Node.ELEMENT_NODE) {
                    Element element = (Element) node;
                    String name = element.getAttribute("name");
                    if (name.equals("n_lines")) {
                        element.setAttribute("value", String.valueOf(nLines));
                    } else if (name.equals("n_columns")) {
                        element.setAttribute("value", String.valueOf(nColumns));
                    } else if (name.equals("n_layers")) {
                        element.setAttribute("value", String.valueOf(nLayers));
                    } else if (name.equals("c_chunk")) {
                        element.setAttribute("value", String.valueOf(cChunk));
                    } else if (name.equals("c_sets")) {
                        element.setAttribute("value", String.valueOf(cSets));
                    } else if (name.equals("c_lines")) {
                        element.setAttribute("value", String.valueOf(cLines));
                    }
                }
            }

            try (FileOutputStream output = new FileOutputStream(modifiedFile)) {
                writeXml(doc, output);
            }
        } catch (ParserConfigurationException | IOException | SAXException | TransformerException e) {
            e.printStackTrace();
        }
    }

    private void displayXmlParameters(DocumentBuilderFactory dbf, String fileName) throws ParserConfigurationException, IOException, org.xml.sax.SAXException {
        DocumentBuilder db = dbf.newDocumentBuilder();
        Document doc = db.parse(new File(fileName));
        doc.getDocumentElement().normalize();

        System.out.println();
        System.out.println("File: " + fileName);

        NodeList list = doc.getElementsByTagName("parameter");
        for (int temp = 0; temp < list.getLength(); temp++) {
            Node node = list.item(temp);
            if (node.getNodeType() == Node.ELEMENT_NODE) {
                Element element = (Element) node;
                String name = element.getAttribute("name");
                String value = element.getAttribute("value");

                System.out.println("---- Simulator Parameters ----");
                System.out.println(name + ": " + value);
            }
        }
    }

    private void displayOutputXml(DocumentBuilderFactory dbf, String fileName) throws ParserConfigurationException, IOException, org.xml.sax.SAXException {
        DocumentBuilder db = dbf.newDocumentBuilder();
        Document doc = db.parse(new File(fileName));
        doc.getDocumentElement().normalize();

        System.out.println();
        System.out.println("File: " + fileName);

        NodeList list = doc.getElementsByTagName("result");
        for (int temp = 0; temp < list.getLength(); temp++) {
            Node node = list.item(temp);
            if (node.getNodeType() == Node.ELEMENT_NODE) {
                Element element = (Element) node;
                String ipc = element.getElementsByTagName("ipc").item(0).getTextContent();
                String energy = element.getElementsByTagName("energy").item(0).getTextContent();

                System.out.println("---- Results ----");
                System.out.println("IPC: " + ipc);
                System.out.println("Energy: " + energy);
            }
        }
    }

    private static void callSimulator(String configPath, String resultPath) {
        ProcessBuilder builder = new ProcessBuilder();
        builder.command("cmd.exe", "/c" + "SimALU " + configPath.trim() + " " + resultPath.trim());
        builder.directory(new File("C:\\Users\\Ana\\Desktop\\FACULTATE\\Dissertation\\simulator"));
        Process process = null;

        try {
            process = builder.start();
        } catch (IOException e) {
            e.printStackTrace();
        }
        try {
            int exitCode = process.waitFor();
            System.out.println("Simulator exit code: " + exitCode);
            process.destroy();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static void writeXml(Document doc, FileOutputStream output) throws TransformerException {
        TransformerFactory transformerFactory = TransformerFactory.newInstance();
        Transformer transformer = transformerFactory.newTransformer();
        transformer.setOutputProperty(OutputKeys.INDENT, "yes");
        transformer.transform(new DOMSource(doc), new StreamResult(output));
    }
}